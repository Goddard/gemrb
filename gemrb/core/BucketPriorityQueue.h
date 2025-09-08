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

#include <cmath>
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
		// store the new point in the bucket with index equals to its cost (rounded to zero), which
		// speeds up the search for the cheapest point
		const int bucketIdx = std::min(static_cast<int>(cost), buckets.bucketsCount - 1);
		minBucket = std::min(minBucket, bucketIdx);
		buckets.PushPoint(bucketIdx, point, cost);
	}

	Point Pop()
	{
		--count;
		// find the first non-empty bucket with minimum index - under this index we will find bucket with the cheapest Point
		while (minBucket < buckets.bucketsCount && buckets.IsEmpty(minBucket)) {
			++minBucket;
		}

		// if you want to use this class outside our pathfinder, you should check HERE if the item even exists;
		// we don't do this, because before every Pop() call we check whether the priority queue isn't empty

		// find the minimum cost index inside this bucket (e.g. Points with Cost 11.0, 11.5 and 11.75 will all land in the bucket with index 11);
		// use linear search, it's the most effective for the low elements count, and we will have typically under 10 elements in the bucket
		const auto costData = buckets.CostData(minBucket);
		int32_t minIdx = 0;
		for (int i = 1; i < buckets.Size(minBucket); ++i) {
			minIdx = costData[i] < costData[minIdx] ? i : minIdx;
		}
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
			return bucketSize[bucketIdx] == 0;
		}

		int Size(const int32_t bucketIdx) const
		{
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
			const auto storageNewLastUsedIdx = GetBucketBeginIdx(bucketIdx) + bucketSize[bucketIdx];
			storageCosts[storageNewLastUsedIdx] = cost;
			storagePoints[storageNewLastUsedIdx] = point;
			++bucketSize[bucketIdx];
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
