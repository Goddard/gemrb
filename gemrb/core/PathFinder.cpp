/* GemRB - Engine Made with preRendered Background
 * Copyright (C) 2020 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

// This file implements the pathfinding logic for actors
// The main logic is in Map::FindPath, which is an
// implementation of the Theta* algorithm, see Daniel et al., 2010
// GemRB uses two overlaid representation of the world: the searchmap and the navmap.
// Pathfinding is done on the searchmap and movement is done on the navmap.
// The navmap is bigger than the searchmap by a factor of (16, 12) on the (x, y) axes.
// Traditional, A* based pathfinding done on the searchmap would constrain movement
// to 45-degree angles and not take advantage of the navmap's higher resolution.
// Compared to A*, Theta* relaxes the constraint that two subsequent nodes in a
// path should be adjacent, only requiring them to be visible and for a straight-line
// path to exist. This allows for actors to move at any angle instead of being constrained
// by the searchmap grid. This also means that some paths are shorter than those found
// by A*.
// Moving to each node in the path thus becomes an automatic regulation problem
// which is solved with a P regulator, see Scriptable.cpp

#include "PathFinder.h"

#include "BucketPriorityQueue.h"
#include "Debug.h"
#include "GameData.h"
#include "Geometry.h"
#include "Interface.h"
#include "Map.h"
#include "RNG.h"

#include "Logging/Logging.h"
#include "Scriptable/Actor.h"

#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

namespace GemRB {

constexpr size_t DEGREES_OF_FREEDOM = 4;
constexpr size_t RAND_DEGREES_OF_FREEDOM = 16;
// SEARCHMAP_SQUARE_DIAGONAL scaled by UpScaleFactor
static unsigned int GetSearchmapSquareDiagonal()
{
	return 20 * core->config.UpScaleFactor;
}
constexpr std::array<char, DEGREES_OF_FREEDOM> dxAdjacent { { 1, 0, -1, 0 } };
constexpr std::array<char, DEGREES_OF_FREEDOM> dyAdjacent { { 0, 1, 0, -1 } };

// Cosines
constexpr std::array<float_t, RAND_DEGREES_OF_FREEDOM> dxRand { { 0.000, -0.383, -0.707, -0.924, -1.000, -0.924, -0.707, -0.383, 0.000, 0.383, 0.707, 0.924, 1.000, 0.924, 0.707, 0.383 } };
// Sines
constexpr std::array<float_t, RAND_DEGREES_OF_FREEDOM> dyRand { { 1.000, 0.924, 0.707, 0.383, 0.000, -0.383, -0.707, -0.924, -1.000, -0.924, -0.707, -0.383, 0.000, 0.383, 0.707, 0.924 } };

// Find the best path of limited length that brings us the farthest from d
Path Map::RunAway(const Point& s, const Point& d, int maxPathLength, bool backAway, const Actor* caller)
{
	if (!caller || !caller->GetSpeed()) return {};
	Point p = s;
	float_t dx = s.x - d.x;
	float_t dy = s.y - d.y;
	char xSign = 1, ySign = 1;
	size_t tries = 0;
	NormalizeDeltas(dx, dy, float_t(gamedata->GetStepTime()) / caller->GetSpeed());
	if (std::abs(dx) <= 0.333 && std::abs(dy) <= 0.333) return {};
	while (SquaredDistance(p, s) < unsigned(maxPathLength * maxPathLength * GetSearchmapSquareDiagonal() * GetSearchmapSquareDiagonal())) {
		Point rad(std::lround(p.x + 3 * xSign * dx), std::lround(p.y + 3 * ySign * dy));
		if (!(GetBlockedInRadius(rad, caller->circleSize) & PathMapFlags::PASSABLE)) {
			tries++;
			// Give up and call the pathfinder if backed into a corner
			// should we return nullptr instead, so we don't accidentally get closer to d?
			// it matches more closely the iwd beetles in ar1015, but is too restrictive — then they can't move at all
			int ups = core->config.UpScaleFactor;
			if (tries > (RAND_DEGREES_OF_FREEDOM * ups)) break;
			// Random rotation
			xSign = RandomFlip() ? -1 : 1;
			ySign = RandomFlip() ? -1 : 1;
			continue;
		}
		p = rad;
	}
	int flags = PF_SIGHT;
	if (backAway) flags |= PF_BACKAWAY;
	return FindPath(s, p, caller->circleSize, caller->circleSize, flags, caller);
}

PathNode Map::RandomWalk(const Point& s, int size, int radius, const Actor* caller) const
{
	if (!caller || !caller->GetSpeed()) return {};
	NavmapPoint p = s;
	int ups = core->config.UpScaleFactor;
	size_t i = RAND<size_t>(0, (RAND_DEGREES_OF_FREEDOM * ups) - 1);
	float_t dx = 3 * dxRand[i % RAND_DEGREES_OF_FREEDOM];
	float_t dy = 3 * dyRand[i % RAND_DEGREES_OF_FREEDOM];

	NormalizeDeltas(dx, dy, float_t(gamedata->GetStepTime()) / caller->GetSpeed());
	size_t tries = 0;
	while (SquaredDistance(p, s) < unsigned(radius * radius * GetSearchmapSquareDiagonal() * GetSearchmapSquareDiagonal())) {
		if (!(GetBlockedInRadius(p + Point(dx, dy), size) & PathMapFlags::PASSABLE)) {
			tries++;
			// Give up if backed into a corner
			if (tries > (RAND_DEGREES_OF_FREEDOM * ups)) {
				return {};
			}
			// Random rotation
			i = RAND<size_t>(0, (RAND_DEGREES_OF_FREEDOM * ups) - 1);
			dx = 3 * dxRand[i % RAND_DEGREES_OF_FREEDOM];
			dy = 3 * dyRand[i % RAND_DEGREES_OF_FREEDOM];
			NormalizeDeltas(dx, dy, float_t(gamedata->GetStepTime()) / caller->GetSpeed());
			p = s;
		} else {
			p.x += dx;
			p.y += dy;
		}
	}
	while (!(GetBlockedInRadius(p + Point(dx, dy), size) & (PathMapFlags::PASSABLE | PathMapFlags::ACTOR))) {
		p.x -= dx;
		p.y -= dy;
	}
	PathNode randomStep;
	const Size& mapSize = PropsSize();
	randomStep.point = Clamp(p, Point(1, 1), Point((mapSize.w - 1) * 16, (mapSize.h - 1) * 12));
	randomStep.orient = GetOrient(s, p);
	return randomStep;
}

Path Map::GetLinePath(const Point& start, int Steps, orient_t Orientation, int flags) const
{
	Point dest = start;

	float_t xoff, yoff, mult;
	if (Orientation <= 4) {
		xoff = -Orientation / 4.0;
	} else if (Orientation <= 12) {
		xoff = -1.0 + (Orientation - 4) / 4.0;
	} else {
		xoff = 1.0 - (Orientation - 12) / 4.0;
	}

	if (Orientation <= 8) {
		yoff = 1.0 - Orientation / 4.0;
	} else {
		yoff = -1.0 + (Orientation - 8) / 4.0;
	}

	mult = 1.0 / std::max(std::fabs(xoff), std::fabs(yoff));

	dest.x += Steps * mult * xoff + 0.5;
	dest.y += Steps * mult * yoff + 0.5;

	return GetLinePath(start, dest, 2, Orientation, flags);
}

Path Map::GetLinePath(const Point& start, const Point& dest, int Speed, orient_t Orientation, int flags) const
{
	int Count = 0;
	int Max = Distance(start, dest);
	Point diff = dest - start;
	Path path;
	path.nodes.reserve(Max);
	path.AppendStep(PathNode { start, Orientation });
	auto StartNode = path.begin();
	for (int Steps = 0; Steps < Max; Steps++) {
		Point p;
		p.x = start.x + (diff.x * Steps / Max);
		p.y = start.y + (diff.y * Steps / Max);

		//the path ends here as it would go off the screen, causing problems
		//maybe there is a better way, but i needed a quick hack to fix
		//the crash in projectiles
		if (p.x < 0 || p.y < 0) {
			return path;
		}

		const Size& mapSize = PropsSize();
		if (p.x > mapSize.w * 16 || p.y > mapSize.h * 12) {
			return path;
		}

		if (!Count) {
			StartNode = path.AppendStep({ p, Orientation });
			Count = Speed;
		} else {
			Count--;
			StartNode->point = p;
			StartNode->orient = Orientation;
		}

		bool wall = bool(GetBlocked(p) & (PathMapFlags::DOOR_IMPASSABLE | PathMapFlags::SIDEWALL));
		if (wall) switch (flags) {
				case GL_REBOUND:
					Orientation = ReflectOrientation(Orientation);
					// TODO: recalculate dest (mirror it)
					break;
				case GL_PASS:
					break;
				default: //premature end
					return path;
			}
	}

	return path;
}

PathNode Map::GetLineEnd(const Point& p, int steps, orient_t orient) const
{
	PathNode lineEnd;
	lineEnd.point.x = p.x + steps * GetSearchmapSquareDiagonal() * dxRand[static_cast<size_t>(orient) % RAND_DEGREES_OF_FREEDOM];
	lineEnd.point.y = p.y + steps * GetSearchmapSquareDiagonal() * dyRand[static_cast<size_t>(orient) % RAND_DEGREES_OF_FREEDOM];
	const Size& mapSize = PropsSize();
	lineEnd.point = Clamp(lineEnd.point, Point(1, 1), Point((mapSize.w - 1) * 16, (mapSize.h - 1) * 12));
	lineEnd.orient = GetOrient(p, lineEnd.point);
	return lineEnd;
}

// Find a path from start to goal, ending at the specified distance from the
// target (the goal must be in sight of the end, if PF_SIGHT is specified)
Path Map::FindPath(const Point& s, const Point& d, const unsigned int size, unsigned int minDistance, int flags, const Actor* caller)
{
	TRACY(ZoneScoped);

	traversabilityCache.Update();

	if (InDebugMode(DebugMode::PATHFINDER))
		Log(DEBUG, "FindPath", "s = {}, d = {}, caller = {}, dist = {}, size = {}",
		    s, d,
		    fmt::WideToChar { caller ? caller->GetShortName() : u"nullptr" },
		    minDistance, size);
	const bool actorsAreBlocking = flags & PF_ACTORS_ARE_BLOCKING;
	const auto blockingTraversabilityValue = actorsAreBlocking ? TraversabilityCache::TraversabilityCellValueActor : TraversabilityCache::TraversabilityCellValueActorNonTraversable;

	// Get upscale factor early for optimizations
	int ups = core->config.UpScaleFactor;

	// TODO: we could optimize this function further by doing everything in SearchmapPoint and converting at the end
	SearchmapPoint smptDest0 { d };
	NavmapPoint nmptDest = d;
	NavmapPoint nmptSource = s;
	if (!(GetBlockedInRadiusTile(smptDest0, size) & PathMapFlags::PASSABLE)) {
		// If the desired target is blocked, find the path
		// to the nearest reachable point.
		// Also avoid bumping a still actor out of its position,
		// but stop just before it
		orient_t direction = GetOrient(nmptDest, nmptSource);
		AdjustPositionDirected(nmptDest, direction, size);
	}

	if (nmptDest == nmptSource) return {};

	SearchmapPoint smptSource { nmptSource };
	SearchmapPoint smptDest { nmptDest };

	if (minDistance < size && !(GetBlockedInRadiusTile(smptDest, size) & (PathMapFlags::PASSABLE | PathMapFlags::ACTOR))) {
		Log(DEBUG, "FindPath", "{} can't fit in destination", fmt::WideToChar { caller ? caller->GetShortName() : u"nullptr" });
		return {};
	}

	// Fast path optimization: if source and destination have direct line of sight, create simple path
	// This is especially beneficial for large upscaled maps where A* becomes expensive
	if (ups > 1 && !(flags & PF_ACTORS_ARE_BLOCKING) && IsWalkableTo(nmptSource, nmptDest, false, caller)) {
		const unsigned int directDist = Distance(nmptSource, nmptDest);
		if (minDistance == 0 || directDist >= minDistance) {
			Path directPath;
			directPath.AppendStep({ nmptSource, GetOrient(nmptSource, nmptDest) });
			directPath.AppendStep({ nmptDest, GetOrient(nmptSource, nmptDest) });
			return directPath;
		}
	}

	const Size& mapSize = PropsSize();
	if (!mapSize.PointInside(smptSource)) return {};

	const auto getChildBlockedStatusFn = size > 2 ? &Map::GetChildBlockedStatusForBigSize : &Map::GetChildBlockedStatusForSmallSize;

	// Initialize data structures
	const size_t mapCellsCount = mapSize.Area();

	// make most data storage for this algorithm static, to avoid memory allocations;
	// each run we just clear the storage, which is keeping the underlying allocated memory at hand
	static BucketPriorityQueue* open = nullptr;
	static Size lastOpenMapSize { 0, 0 };
	static std::vector<bool> isClosed;
	static std::vector<NavmapPoint> parents;
	static std::vector<unsigned short> distFromStart;
	static std::mutex pathfindingMutex;

	// Lock to prevent simultaneous pathfinding operations from interfering
	std::lock_guard<std::mutex> lock(pathfindingMutex);

	// Initialize or recreate priority queue if map size changed
	// Recreate if not present or if map size changed (scale depends on map)
	if (!open || lastOpenMapSize != mapSize) {
		delete open;
		open = new BucketPriorityQueue(mapSize.w, mapSize.h);
		lastOpenMapSize = mapSize;
	}

	// Pre-size data structures based on map size to avoid repeated reallocations
	// Use a slightly larger size to account for potential growth
	const size_t optimalSize = mapCellsCount + (mapCellsCount / 8); // 12.5% buffer
	if (isClosed.size() != optimalSize) {
		parents.resize(optimalSize);
		distFromStart.resize(optimalSize);
		isClosed.resize(optimalSize);
	}

	// cleanup
	open->Clear();

	// Only clear the portion we'll actually use to improve cache efficiency
	std::fill(isClosed.begin(), isClosed.begin() + mapCellsCount, false);

	// Use fast memset for arrays since we know the exact size
	memset(static_cast<void*>(parents.data()), 0, sizeof(decltype(parents)::value_type) * mapCellsCount);
	memset(static_cast<void*>(distFromStart.data()), 0xFF, sizeof(decltype(distFromStart)::value_type) * mapCellsCount); // 0xFF sets to max value for unsigned short

	// begin algo init
	distFromStart[smptSource.y * mapSize.w + smptSource.x] = 0;
	parents[smptSource.y * mapSize.w + smptSource.x] = nmptSource;

	open->Push(nmptSource, 0);

	bool foundPath = false;
	// Don't scale squaredMinDist - keep it in navmap coordinates to match SquaredDistance calculations
	unsigned int squaredMinDist = minDistance * minDistance;

	// Bound iterations by map area; exploring more than the whole map isn't useful, and avoids premature aborts
	// Scale iteration limit more conservatively for large upscaled maps to prevent excessive computation
	const int baseIterations = static_cast<int>(mapCellsCount);
	const int maxIterations = ups > 2 ? std::min(baseIterations, baseIterations / (ups / 2)) : baseIterations;
	int iterations = 0;

	// Heuristic weight: increase slightly for large maps to bias toward goal direction and reduce search space
	const float_t HEURISTIC_WEIGHT = ups > 2 ? 1.5f : 1.25f;

	const auto getHeuristic = [&](const SearchmapPoint& smptChild, const int& smptChildIdx) {
		// Calculate heuristic using faster integer distance approximation for performance
		const int xDist = smptChild.x - smptDest.x;
		const int yDist = smptChild.y - smptDest.y;
		// Use faster Manhattan + diagonal distance approximation instead of Euclidean
		const int absX = std::abs(xDist);
		const int absY = std::abs(yDist);
		const float distance = std::max(absX, absY) + 0.414f * std::min(absX, absY);

		// Simplified tie-breaking for performance - only apply on smaller maps
		float heuristic;
		if (ups <= 2) {
			const int dxCross = smptDest.x - smptSource.x;
			const int dyCross = smptDest.y - smptSource.y;
			const int crossProduct = std::abs(xDist * dyCross - yDist * dxCross) >> 3;
			heuristic = HEURISTIC_WEIGHT * (distance + crossProduct);
		} else {
			heuristic = HEURISTIC_WEIGHT * distance;
		}
		const float estDist = distFromStart[smptChildIdx] + heuristic;
		return estDist;
	};

	// Penalize nodes that are close to impassable tiles to keep waypoints away from walls
	// Uses a small Chebyshev radius ring check on the searchmap. Low-cost, local, and effective.
	const auto nearWallPenalty = [&](const SearchmapPoint& smpt) -> unsigned short {
		// Larger maps get a slightly larger clearance radius and penalty
		const int maxR = (ups > 2) ? 2 : 1; // in searchmap cells
		for (int r = 1; r <= maxR; ++r) {
			for (int dy = -r; dy <= r; ++dy) {
				for (int dx = -r; dx <= r; ++dx) {
					if (std::max(std::abs(dx), std::abs(dy)) != r) continue; // ring only
					const int nx = smpt.x + dx;
					const int ny = smpt.y + dy;
					if (nx < 0 || ny < 0 || nx >= mapSize.w || ny >= mapSize.h) continue;
					const SearchmapPoint around { nx, ny };
					const PathMapFlags aroundFlags = (this->*getChildBlockedStatusFn)(around, size);
					const bool aroundBlocked = !(aroundFlags & (PathMapFlags::PASSABLE | PathMapFlags::ACTOR));
					if (aroundBlocked) {
						// Higher penalty the closer we are; base step scaled slightly for upscaled maps
						const unsigned short base = (ups > 2) ? 2 : 1;
						return static_cast<unsigned short>(base * (maxR - (r - 1)));
					}
				}
			}
		}
		return 0;
	};

	while (!open->IsEmpty() && iterations < maxIterations) {
		++iterations;
		const NavmapPoint nmptCurrent = open->Pop();

		const SearchmapPoint smptCurrent { nmptCurrent };
		const int smptCurrentIdx = smptCurrent.y * mapSize.w + smptCurrent.x;
		if (parents[smptCurrentIdx].IsZero()) {
			continue;
		}

		if (smptCurrent == smptDest) {
			nmptDest = nmptCurrent;
			foundPath = true;
			break;
		}

		if (minDistance &&
		    parents[smptCurrentIdx] != nmptCurrent &&
		    SquaredDistance(nmptCurrent, nmptDest) < squaredMinDist && // More conservative termination
		    (!(flags & PF_SIGHT) || IsVisibleLOS(nmptCurrent, nmptDest, caller))) { // Use NavmapPoint coordinates for consistency
			smptDest = smptCurrent;
			nmptDest = nmptCurrent;
			foundPath = true;
			break;
		}

		isClosed[smptCurrentIdx] = true;
		for (size_t i = 0; i < DEGREES_OF_FREEDOM; i++) {
			const NavmapPoint nmptChild(nmptCurrent.x + 16 * dxAdjacent[i], nmptCurrent.y + 12 * dyAdjacent[i]);
			const SearchmapPoint smptChild { nmptChild };
			// Outside map
			if (smptChild.x < 0 || smptChild.y < 0 || smptChild.x >= mapSize.w || smptChild.y >= mapSize.h) continue;
			// Already visited
			int smptChildIdx = smptChild.y * mapSize.w + smptChild.x;
			if (isClosed[smptChildIdx]) continue;

			const PathMapFlags childBlockStatus = (this->*getChildBlockedStatusFn)(smptChild, size);
			bool childBlocked = !(childBlockStatus & (PathMapFlags::PASSABLE | PathMapFlags::ACTOR));
			if (childBlocked) continue;

			// If there's an actor, check it can be bumped away
			const auto navmapCellTraversability = traversabilityCache.GetCellData(nmptChild.y * mapSize.w * 16 + nmptChild.x);
			const bool childIsUnbumpable = navmapCellTraversability.occupyingActor != caller && navmapCellTraversability.state >= blockingTraversabilityValue;
			if (childIsUnbumpable) continue;

			SearchmapPoint smptCurrent2 { nmptCurrent };
			NavmapPoint nmptParent = parents[smptCurrent2.y * mapSize.w + smptCurrent2.x];
			SearchmapPoint smptParent { nmptParent };
			unsigned short oldDist = distFromStart[smptChildIdx];

			// Lazy Theta star*
			unsigned short newDist = static_cast<unsigned short>(distFromStart[smptParent.y * mapSize.w + smptParent.x] + Distance(smptParent, smptChild));
			unsigned short penalizedDist = static_cast<unsigned short>(newDist + nearWallPenalty(smptChild));
			if (penalizedDist < oldDist) {
				parents[smptChildIdx] = nmptParent;
				distFromStart[smptChildIdx] = penalizedDist;
			}

			if (distFromStart[smptChildIdx] < oldDist) {
				// Theta-star path if there is LOS
				// On very large upscaled maps, limit Theta* to nearby nodes for performance
				const bool performThetaStar = (ups <= 3) || (Distance(nmptParent, nmptChild) < static_cast<unsigned int>(20 * ups));

				if (performThetaStar && !IsWalkableTo(nmptParent, nmptChild, actorsAreBlocking, caller)) {
					// Fall back to A-star path
					distFromStart[smptChildIdx] = std::numeric_limits<unsigned short>::max();
					// Find already visited neighbour with shortest: path from start + path to child
					for (size_t j = 0; j < DEGREES_OF_FREEDOM; j++) {
						NavmapPoint nmptVis(nmptChild.x + 16 * dxAdjacent[j], nmptChild.y + 12 * dyAdjacent[j]);
						SearchmapPoint smptVis { nmptVis };
						// Outside map
						if (smptVis.x < 0 || smptVis.y < 0 || smptVis.x >= mapSize.w || smptVis.y >= mapSize.h) continue;
						// Only consider already visited
						if (!isClosed[smptVis.y * mapSize.w + smptVis.x]) continue;

						unsigned short oldVisDist = distFromStart[smptChildIdx];
						newDist = static_cast<unsigned short>(distFromStart[smptVis.y * mapSize.w + smptVis.x] + Distance(smptVis, smptChild));
						penalizedDist = static_cast<unsigned short>(newDist + nearWallPenalty(smptChild));
						if (penalizedDist < oldVisDist) {
							parents[smptChildIdx] = nmptVis;
							distFromStart[smptChildIdx] = penalizedDist;
						}
					}
					if (distFromStart[smptChildIdx] >= oldDist) continue;
				} else if (!performThetaStar) {
					// Skip Theta* optimization for distant nodes on large maps, fall back to A*
					distFromStart[smptChildIdx] = std::numeric_limits<unsigned short>::max();
					newDist = static_cast<unsigned short>(distFromStart[smptCurrent2.y * mapSize.w + smptCurrent2.x] + Distance(smptCurrent2, smptChild));
					penalizedDist = static_cast<unsigned short>(newDist + nearWallPenalty(smptChild));
					if (penalizedDist < oldDist) {
						parents[smptChildIdx] = nmptCurrent;
						distFromStart[smptChildIdx] = penalizedDist;
					}
					if (distFromStart[smptChildIdx] >= oldDist) continue;
				}

				const float newCost = getHeuristic(smptChild, smptChildIdx);
				open->Push(nmptChild, newCost);
			}
		}
	}

	if (foundPath) {
		// If we're on an upscaled map, post-process the reconstructed chain to reduce
		// waypoint density while preserving line-of-sight and keeping a reasonable spacing.
		if (ups > 1) {
			// 1) Reconstruct full chain from source -> destination
			std::vector<NavmapPoint> chain;
			{
				std::vector<NavmapPoint> rev;
				NavmapPoint cur = nmptDest;
				SearchmapPoint smptCur { cur };
				while (true) {
					rev.push_back(cur);
					NavmapPoint par = parents[smptCur.y * mapSize.w + smptCur.x];
					if (cur == par) break; // reached source (parent points to itself)
					cur = par;
					smptCur = SearchmapPoint(cur);
				}
				// reverse to get source -> dest order
				chain.assign(rev.rbegin(), rev.rend());
			}

			// 2) LOS-based compression with minimum spacing
			std::vector<NavmapPoint> simplified;
			simplified.reserve(chain.size());
			if (!chain.empty()) simplified.push_back(chain.front());

			// Minimum spacing scaled with map upscale; slightly larger for big ups
			const float spacingMult = (ups > 2) ? 1.25f : 1.0f;
			const unsigned int minSpacing = static_cast<unsigned int>(GetSearchmapSquareDiagonal() * spacingMult);

			size_t lastKeptIdx = 0;
			while (lastKeptIdx + 1 < chain.size()) {
				size_t farIdx = lastKeptIdx + 1;
				// Extend as far as LOS allows
				for (size_t k = farIdx + 1; k < chain.size(); ++k) {
					if (IsWalkableTo(chain[lastKeptIdx], chain[k], actorsAreBlocking, caller)) {
						farIdx = k;
					} else {
						break;
					}
				}

				// Try to ensure reasonable spacing; if jump is tiny, see if we can pick a farther visible point
				size_t targetIdx = farIdx;
				if (Distance(chain[lastKeptIdx], chain[targetIdx]) < minSpacing) {
					for (size_t k = farIdx + 1; k < chain.size(); ++k) {
						if (!IsWalkableTo(chain[lastKeptIdx], chain[k], actorsAreBlocking, caller)) break;
						if (Distance(chain[lastKeptIdx], chain[k]) >= minSpacing) {
							targetIdx = k;
						} else {
							// keep searching, but remember the last LOS-valid
							targetIdx = k;
						}
					}
				}

				simplified.push_back(chain[targetIdx]);
				lastKeptIdx = targetIdx;
			}

			// 3) Build final Path with orientations
			// If compression results in no actual movement, return empty path
			if (simplified.size() < 2) {
				return {};
			}

			Path resultPath;
			for (size_t i = 1; i < simplified.size(); ++i) {
				const NavmapPoint& prev = simplified[i - 1];
				const NavmapPoint& curr = simplified[i];
				PathNode step { curr, S };
				if ((flags & PF_BACKAWAY) && i == 1) {
					// preserve back-away behavior for the first movement segment
					step.orient = GetOrient(curr, prev);
				} else {
					step.orient = GetOrient(prev, curr);
				}
				resultPath.AppendStep(std::move(step));
			}

			if (InDebugMode(DebugMode::PATHFINDER)) {
				Log(DEBUG, "FindPath", "Compressed path nodes: {} -> {} (final: {}) (ups = {}, spacing = {}))",
				    chain.size(), simplified.size(), resultPath.Size(), ups, minSpacing);
			}

			return resultPath;
		} else {
			// Original reconstruction for non-upscaled maps (preserve legacy behavior)
			Path resultPath;
			NavmapPoint nmptCurrent = nmptDest;
			NavmapPoint nmptParent;
			SearchmapPoint smptCurrent { nmptCurrent };
			while (!resultPath || nmptCurrent != parents[smptCurrent.y * mapSize.w + smptCurrent.x]) {
				nmptParent = parents[smptCurrent.y * mapSize.w + smptCurrent.x];
				PathNode newStep { nmptCurrent, S };
				// movement in general allows characters to walk backwards given that
				// the destination is behind the character (within a threshold), and
				// that the distance isn't too far away
				// we approximate that with a relaxed collinearity check and intentionally
				// skip the first step, otherwise it doesn't help with iwd beetles in ar1015
				if (flags & PF_BACKAWAY && resultPath && std::abs(area2(nmptCurrent, resultPath.GetStep(0).point, nmptParent)) < 300) {
					newStep.orient = GetOrient(nmptCurrent, nmptParent);
				} else {
					newStep.orient = GetOrient(nmptParent, nmptCurrent);
				}
				resultPath.PrependStep(newStep);
				nmptCurrent = nmptParent;
				smptCurrent = SearchmapPoint(nmptCurrent);
			}
			return resultPath;
		}
	} else if (InDebugMode(DebugMode::PATHFINDER)) {
		if (caller) {
			if (iterations >= maxIterations) {
				Log(DEBUG, "FindPath", "Pathing failed for {} (iteration limit reached: {}/{})",
				    fmt::WideToChar { caller->GetShortName() }, iterations, maxIterations);
			} else {
				Log(DEBUG, "FindPath", "Pathing failed for {}", fmt::WideToChar { caller->GetShortName() });
			}
		} else {
			if (iterations >= maxIterations) {
				Log(DEBUG, "FindPath", "Pathing failed (iteration limit reached: {}/{})", iterations, maxIterations);
			} else {
				Log(DEBUG, "FindPath", "Pathing failed");
			}
		}
	}

	return {};
}

void Map::NormalizeDeltas(float_t& dx, float_t& dy, const float_t factor)
{
	const int ups = core->config.UpScaleFactor;
	constexpr float_t BASE_STEP_RADIUS = 2.0;
	const float_t STEP_RADIUS = BASE_STEP_RADIUS * ups;

	const float_t ySign = std::copysign(1.0f, dy);
	const float_t xSign = std::copysign(1.0f, dx);
	dx = std::fabs(dx);
	dy = std::fabs(dy);
	const float_t dxOrig = dx;
	const float_t dyOrig = dy;
	if (dx == 0.0) {
		dy = STEP_RADIUS * 0.75f;
	} else if (dy == 0.0) {
		dx = STEP_RADIUS;
	} else {
		const float_t q = STEP_RADIUS / std::hypotf(dx, dy);
		dx = dx * q;
		dy = dy * q * 0.75f;
	}
	dx = std::min(dx * factor, dxOrig);
	dy = std::min(dy * factor, dyOrig);
	dx = std::ceil(dx) * xSign;
	dy = std::ceil(dy) * ySign;
}

}
