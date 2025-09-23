/* GemRB - Engine Made with preRendered Background
 * Copyright (C) 2025 The GemRB Project
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

#ifndef BUCKETPRIORITYQUEUE_H
#define BUCKETPRIORITYQUEUE_H


#include "Region.h"

#include "Logging/Logging.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace GemRB {

/**
 * Specialized class for choosing the cheapest Point from the pathfinder's set of 'open' nodes.
 * It's tailor-made to our pathfinder characteristics: buckets' count is selected to fit our Cost values,
 * size of the individual bucket is chosen to fit (with a reasonable big safety margin) all Points with similar cost,
 * yet to remain relatively local in terms of memory, the method for selecting the next item to Pop is also made with
 * our needs in mind (keeping track of the bucket with the lowest used index, use of the linear search within the
 * bucket).
 */
class BucketPriorityQueue {
public:
	BucketPriorityQueue(int mapWidth = 0, int mapHeight = 0) : buckets(mapWidth, mapHeight), minBucket(buckets.bucketsCount), count(0) {}

	void Push(const Point& point, const float cost)
	{
		++count;
		// sanitize cost: protect against NaN/Inf and absurdly large/small values
		float c = cost;
		if (!std::isfinite(c)) {
			Log(WARNING, "BucketPriorityQueue", "Push: non-finite cost {} -> clamping to max bucket", c);
			c = static_cast<float>(buckets.bucketsCount - 1);
		}

		// compute bucket index from cost using scale to limit total buckets
		int bucketIdx = static_cast<int>(std::floor(c / buckets.scale));
		if (bucketIdx < 0) {
			bucketIdx = 0;
		} else if (bucketIdx >= buckets.bucketsCount) {
			// Grow buckets to accommodate this index instead of clamping
			buckets.EnsureBucketIndex(bucketIdx);
		}
		// If this bucket was empty, add it to the active set
		if (buckets.Size(bucketIdx) == 0) {
			activeBuckets.push(bucketIdx);
		}
		minBucket = std::min(minBucket, bucketIdx);
		buckets.PushPoint(bucketIdx, point, c);
	}

	Point Pop()
	{
		// Defensive: ensure the queue actually contains elements before popping.
		if (IsEmpty()) {
			Log(ERROR, "BucketPriorityQueue", "Pop called on empty queue -> returning default Point");
			return Point();
		}
		// Use a min-heap of active (non-empty) bucket indices to avoid scanning empties
		int bucketIdx = -1;
		bool usedHeap = false;
		while (!activeBuckets.empty()) {
			int idx = activeBuckets.top();
			if (!buckets.IsEmpty(idx)) {
				bucketIdx = idx;
				usedHeap = true;
				break;
			}
			activeBuckets.pop(); // discard stale index
		}

		if (bucketIdx < 0) {
			// Fallback: no active bucket tracked; scan from minBucket to recover
			while (minBucket < buckets.bucketsCount && buckets.IsEmpty(minBucket)) {
				++minBucket;
			}
			if (minBucket >= buckets.bucketsCount) {
				Log(ERROR, "BucketPriorityQueue", "Pop: no non-empty buckets found (minBucket {}) -> returning default Point", minBucket);
				return Point();
			}
			bucketIdx = minBucket;
		}

		// select the minimum cost element inside this bucket
		const auto costData = buckets.CostData(bucketIdx);
		int32_t minIdx = 0;
		const int bucketSize = buckets.Size(bucketIdx);

		// Optimization: for large buckets on upscaled maps, use sampling instead of full scan
		if (bucketSize > 16) {
			// Sample every 4th element plus first/last to find approximate minimum
			for (int i = 0; i < bucketSize; i += 4) {
				minIdx = costData[i] < costData[minIdx] ? i : minIdx;
			}
			if (bucketSize > 1) {
				minIdx = costData[bucketSize - 1] < costData[minIdx] ? bucketSize - 1 : minIdx;
			}
			// Refine search around the sampled minimum
			const int start = std::max(0, minIdx - 2);
			const int end = std::min(bucketSize, minIdx + 3);
			for (int i = start; i < end; ++i) {
				minIdx = costData[i] < costData[minIdx] ? i : minIdx;
			}
		} else {
			// Small buckets: use full scan as before
			for (int i = 1; i < bucketSize; ++i) {
				minIdx = costData[i] < costData[minIdx] ? i : minIdx;
			}
		}

		--count;
		Point p = buckets.PopPoint(bucketIdx, minIdx);
		// Update active buckets: if still non-empty, keep it active; otherwise remove and advance minBucket if needed
		if (usedHeap) {
			// We consumed the top entry corresponding to this bucket
			if (!activeBuckets.empty()) activeBuckets.pop();
			if (!buckets.IsEmpty(bucketIdx)) {
				activeBuckets.push(bucketIdx);
			}
		} else {
			// Fallback path: heap wasn't used. If the bucket still contains items, ensure it's tracked.
			if (!buckets.IsEmpty(bucketIdx)) {
				activeBuckets.push(bucketIdx);
			}
		}
		if (bucketIdx == minBucket && buckets.IsEmpty(bucketIdx)) {
			// advance minBucket to next non-empty if possible
			while (minBucket < buckets.bucketsCount && buckets.IsEmpty(minBucket)) {
				++minBucket;
			}
		}
		return p;
	}

	bool IsEmpty() const
	{
		return count <= 0;
	}

	void Clear()
	{
		count = 0;
		minBucket = buckets.bucketsCount;
		buckets.Clear();
		// Clear active buckets heap
		activeBuckets = ActiveMinHeap();
	}

public:
	/**
	 *  Specialized class providing buckets for pathfiding's set of 'open' nodes - a pair of {Point, Cost}, which should
	 *  be accessed based off their lowest cost.
	 *  CostPointBuckets store Points in buckets indexed by their Cost, e.g. a point with cost of 225.75 will land in the
	 *  bucket with index 225 - this guarantees that the cheapest Points will be in the lowest occupied buckets, which
	 *  is used in the BucketPriorityQueue.
	 */
	class CostPointBuckets {
	public:
		// Budget of target buckets we aim to allocate before growth; scaling compresses costs into this range
		// Increase target buckets for large upscaled maps to reduce bucket collisions
		static constexpr int32_t BASE_TARGET_BUCKETS = 16384; // 16k buckets base budget
		static constexpr int32_t EXTRA_CUSHION = 1024; // Extra to reduce early growth

		// Scale target buckets based on map size for better distribution
		static int32_t GetTargetBuckets(int mapWidth, int mapHeight)
		{
			const int mapArea = mapWidth * mapHeight;
			if (mapArea > 10000) { // Large upscaled maps
				return BASE_TARGET_BUCKETS * 2; // 32k buckets for large maps
			} else if (mapArea > 5000) {
				return BASE_TARGET_BUCKETS + BASE_TARGET_BUCKETS / 2; // 24k buckets for medium maps
			} else {
				return BASE_TARGET_BUCKETS; // 16k buckets for small maps
			}
		}

		// Estimate raw maximum cost scale for initial sizing
		static float EstimateRawMaxCost(int mapWidth, int mapHeight)
		{
			const float maxDistance = std::hypotf(mapWidth, mapHeight);
			// distFromStart grows roughly with traveled distance; heuristic adds another distance term
			// Use more conservative multiplier for large maps to avoid excessive bucket growth
			const int mapArea = mapWidth * mapHeight;
			const float multiplier = mapArea > 10000 ? 6.0f : 10.0f; // Reduce multiplier for large maps
			return std::max(1.0f, maxDistance * multiplier) + 2048.0f; // Reduce base offset too
		}

		// Default fallback for compatibility
		constexpr static int32_t DEFAULT_BUCKETS_COUNT = BASE_TARGET_BUCKETS;

		CostPointBuckets(int mapWidth = 0, int mapHeight = 0)
			: bucketsCount(DEFAULT_BUCKETS_COUNT)
		{
			if (mapWidth > 0 && mapHeight > 0) {
				const float rawMax = EstimateRawMaxCost(mapWidth, mapHeight);
				const int targetBuckets = GetTargetBuckets(mapWidth, mapHeight);
				// Choose scale to fit rawMax into our target bucket budget
				scale = std::max(1.0f, std::ceil(rawMax / static_cast<float>(targetBuckets)));
				bucketsCount = static_cast<int32_t>(std::ceil(rawMax / scale)) + EXTRA_CUSHION;
			} else {
				scale = 1.0f;
			}
			InitializeStorage();
		}


		bool IsEmpty(const int32_t bucketIdx) const
		{
			if (bucketIdx < 0 || bucketIdx >= bucketsCount) {
				Log(WARNING, "BucketPriorityQueue", "IsEmpty: bucketIdx {} out of range [0, {}) -> returning true",
				    bucketIdx, bucketsCount);
				return true;
			}
			return points[bucketIdx].empty();
		}

		int Size(const int32_t bucketIdx) const
		{
			if (bucketIdx < 0 || bucketIdx >= bucketsCount) {
				Log(WARNING, "BucketPriorityQueue", "Size: bucketIdx {} out of range [0, {}) -> returning 0",
				    bucketIdx, bucketsCount);
				return 0;
			}
			return static_cast<int>(points[bucketIdx].size());
		}

		Point PopPoint(const int32_t bucketIdx, const int32_t itemIdx)
		{
			// pop item by swapping item from itemIdx with last item in the bucket vectors
			auto& pts = points[bucketIdx];
			auto& cts = costs[bucketIdx];
			const int lastIdx = static_cast<int>(pts.size()) - 1;
			const Point wantedItem = pts[itemIdx];
			if (itemIdx != lastIdx) {
				pts[itemIdx] = pts[lastIdx];
				cts[itemIdx] = cts[lastIdx];
			}
			pts.pop_back();
			cts.pop_back();
			return wantedItem;
		}

		void Clear()
		{
			for (int32_t i = 0; i < bucketsCount; ++i) {
				points[i].clear();
				costs[i].clear();
			}
		}

		void PushPoint(const int32_t bucketIdx, const Point& point, const float cost)
		{
			int32_t idx = bucketIdx;
			if (idx < 0) {
				Log(DEBUG, "BucketPriorityQueue", "PushPoint: negative bucketIdx {} -> clamping to 0", idx);
				idx = 0;
			}
			EnsureBucketIndex(idx);
			points[idx].push_back(point);
			costs[idx].push_back(cost);
		}

		const float* CostData(const int32_t bucketIdx) const
		{
			return costs[bucketIdx].empty() ? nullptr : costs[bucketIdx].data();
		}

	public:
		// Public access to buckets count (mutable to allow growth)
		int32_t bucketsCount;
		// Cost-to-bucket scaling factor
		float scale = 1.0f;

		// Ensure that bucket index 'idx' can be addressed; grows structures if needed
		void EnsureBucketIndex(const int32_t idx)
		{
			if (idx < bucketsCount) return;
			// Grow to fit idx, with some slack to avoid frequent resizes
			int32_t newCount = std::max(idx + 1, bucketsCount + std::max(4096, bucketsCount / 2));
			Log(DEBUG, "BucketPriorityQueue", "Growing buckets from {} to {} to accommodate idx {}", bucketsCount, newCount, idx);
			bucketsCount = newCount;
			points.resize(bucketsCount);
			costs.resize(bucketsCount);
		}

	private:
		void InitializeStorage()
		{
			points.resize(bucketsCount);
			costs.resize(bucketsCount);
		}

		// Dynamic storage: one vector per bucket
		std::vector<std::vector<Point>> points;
		std::vector<std::vector<float>> costs;
	};

	CostPointBuckets buckets;
	int minBucket;
	int count = 0;
	// Min-heap of non-empty bucket indices to avoid scanning across empty buckets
	using ActiveMinHeap = std::priority_queue<int, std::vector<int>, std::greater<int>>;
	ActiveMinHeap activeBuckets;
};

}
#endif // BUCKETPRIORITYQUEUE_H
