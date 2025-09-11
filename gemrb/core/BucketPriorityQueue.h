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
#include <limits>
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

		// compute bucket index from cost (floor), clamp to valid range
		int bucketIdx = static_cast<int>(std::floor(c));
		if (bucketIdx < 0) {
			bucketIdx = 0;
		} else if (bucketIdx >= buckets.bucketsCount) {
			Log(WARNING, "BucketPriorityQueue", "Push: computed bucketIdx {} >= bucketsCount {} -> clamping to last bucket",
			    bucketIdx, buckets.bucketsCount);
			bucketIdx = buckets.bucketsCount - 1;
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

		// find the first non-empty bucket with minimum index - under this index we will find bucket with the cheapest Point
		while (minBucket < buckets.bucketsCount && buckets.IsEmpty(minBucket)) {
			++minBucket;
		}

		// If we reached past the last bucket, nothing to pop; return default Point instead of UB.
		if (minBucket >= buckets.bucketsCount) {
			Log(ERROR, "BucketPriorityQueue", "Pop: no non-empty buckets found (minBucket {}) -> returning default Point", minBucket);
			return Point();
		}

		// find the minimum cost index inside this bucket
		const auto costData = buckets.CostData(minBucket);
		int32_t minIdx = 0;
		for (int i = 1; i < buckets.Size(minBucket); ++i) {
			minIdx = costData[i] < costData[minIdx] ? i : minIdx;
		}

		--count;
		return buckets.PopPoint(minBucket, minIdx);
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
		// Dynamic sizing based on map size to prevent buffer overflows
		// Calculate maximum expected cost based on map diagonal distance
		static int32_t CalculateBucketsCount(int mapWidth, int mapHeight)
		{
			// Estimate maximum cost: map diagonal * heuristic weight * safety factor
			const float maxDistance = std::hypotf(mapWidth, mapHeight);
			const float maxCost = maxDistance * 5.0f; // Increased safety factor for accumulated costs
			return static_cast<int32_t>(std::ceil(maxCost)) + 2000; // Extra 2000 buckets for safety
		}

		// Default fallback for compatibility
		constexpr static int32_t DEFAULT_BUCKETS_COUNT = 1024 * 5;

		CostPointBuckets(int mapWidth = 0, int mapHeight = 0)
			: bucketsCount(mapWidth > 0 && mapHeight > 0 ? CalculateBucketsCount(mapWidth, mapHeight) : DEFAULT_BUCKETS_COUNT)
		{
			InitializeStorage();
		}


		bool IsEmpty(const int32_t bucketIdx) const
		{
			if (bucketIdx < 0 || bucketIdx >= bucketsCount) {
				Log(WARNING, "BucketPriorityQueue", "IsEmpty: bucketIdx {} out of range [0, {}) -> returning true",
				    bucketIdx, bucketsCount);
				return true;
			}
			return bucketSize[bucketIdx] == 0;
		}

		int Size(const int32_t bucketIdx) const
		{
			if (bucketIdx < 0 || bucketIdx >= bucketsCount) {
				Log(WARNING, "BucketPriorityQueue", "Size: bucketIdx {} out of range [0, {}) -> returning 0",
				    bucketIdx, bucketsCount);
				return 0;
			}
			return bucketSize[bucketIdx];
		}

		Point PopPoint(const int32_t bucketIdx, const int32_t itemIdx)
		{
			// pop item by swapping item from itemIdx with last item from the bucket
			--bucketSize[bucketIdx];
			const auto storageLastUsedIdx = GetBucketBeginIdx(bucketIdx) + bucketSize[bucketIdx];
			const auto storageItemIdx = GetBucketBeginIdx(bucketIdx) + itemIdx;
			const Point wantedItem = storagePoints[storageItemIdx];
			storageCosts[storageItemIdx] = storageCosts[storageLastUsedIdx];
			storagePoints[storageItemIdx] = storagePoints[storageLastUsedIdx];
			return wantedItem;
		}

		void Clear()
		{
			// zero only the bucketSize, we don't care if the values from storage are
			// zeroed if bucketSize tells the bucket is empty
			std::memset(bucketSize.data(), 0, bucketsCount * sizeof(bucketSize[0]));
		}

		void PushPoint(const int32_t bucketIdx, const Point& point, const float cost)
		{
			// defensive checks: ensure bucketIdx within expected range
			int32_t idx = bucketIdx;
			if (idx < 0) {
				Log(DEBUG, "BucketPriorityQueue", "PushPoint: negative bucketIdx {} -> clamping to 0", idx);
				idx = 0;
			} else if (idx >= bucketsCount) {
				Log(WARNING, "BucketPriorityQueue", "PushPoint: bucketIdx {} >= bucketsCount {} -> clamping to last bucket",
				    idx, bucketsCount);
				idx = bucketsCount - 1;
			}

			// ensure bucketSize vector can be indexed safely
			if (static_cast<size_t>(idx) >= bucketSize.size()) {
				Log(ERROR, "BucketPriorityQueue", "PushPoint: internal bucketSize vector too small (idx {} >= size {}) - resizing bucketSize",
				    idx, bucketSize.size());
				bucketSize.resize(bucketsCount);
			}

			const auto storageNewLastUsedIdx = GetBucketBeginIdx(idx) + bucketSize[idx];

			// if storage vectors are unexpectedly small, resize them instead of crashing
			const size_t requiredIndex = static_cast<size_t>(storageNewLastUsedIdx);
			const size_t neededStorage = static_cast<size_t>(GetBucketBeginIdx(idx + 1));
			// ensure we resize to cover the requiredIndex (which will be written) => requiredIndex + 1
			if (requiredIndex >= storageCosts.size() || neededStorage > storageCosts.size()) {
				// exponential growth strategy: double current size until it fits or use neededStorage
				size_t newSize = std::max(storageCosts.size() ? storageCosts.size() : static_cast<size_t>(1), static_cast<size_t>(1));
				while (newSize <= requiredIndex || newSize < neededStorage) {
					newSize = newSize * 2;
				}
				newSize = std::max(newSize, requiredIndex + 1);
				newSize = std::max(newSize, neededStorage);
				Log(WARNING, "BucketPriorityQueue", "PushPoint: storage index {} or needed {} >= storage size {} - resizing storage to {}",
				    storageNewLastUsedIdx, neededStorage, storageCosts.size(), newSize);
				storageCosts.resize(newSize);
				storagePoints.resize(newSize);
			}

			storageCosts[storageNewLastUsedIdx] = cost;
			storagePoints[storageNewLastUsedIdx] = point;
			++bucketSize[idx];
		}

		const float* CostData(const int32_t bucketIdx) const
		{
			return &storageCosts[GetBucketBeginIdx(bucketIdx)];
		}

	public:
		// Public access to buckets count
		const int32_t bucketsCount;

	private:
		constexpr static int32_t BUCKET_SIZE = 100;

		void InitializeStorage()
		{
			storagePoints.resize(BUCKET_SIZE * bucketsCount);
			storageCosts.resize(BUCKET_SIZE * bucketsCount);
			bucketSize.resize(bucketsCount);
			Clear();
		}

		static int32_t GetBucketBeginIdx(const int32_t bucketIdx)
		{
			return BUCKET_SIZE * bucketIdx;
		}

		// Dynamic storage that scales with map size
		std::vector<Point> storagePoints;
		std::vector<float> storageCosts;
		std::vector<uint8_t> bucketSize;
	};

	CostPointBuckets buckets;
	int minBucket;
	int count = 0;
};

}
#endif // BUCKETPRIORITYQUEUE_H
