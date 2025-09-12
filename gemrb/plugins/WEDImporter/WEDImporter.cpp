/* GemRB - Infinity Engine Emulator
 * Copyright (C) 2003 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

#include "WEDImporter.h"

#include "GameData.h"
#include "Interface.h"
#include "PluginMgr.h"

#include "Logging/Logging.h"
#include "Plugins/TileSetMgr.h"

#include <iterator>

using namespace GemRB;

//the net sizeof(wed_polygon) is 0x12 but not all compilers know that
#define WED_POLYGON_SIZE 0x12

WEDImporter::~WEDImporter(void)
{
	delete str;
}

bool WEDImporter::Open(DataStream* stream)
{
	if (stream == NULL) {
		return false;
	}
	delete str;
	str = stream;
	// Accept both legacy 8-byte signature ("WED V1.3") and split sig+ver ("WED ", "V1.3"/"V1.4")
	char Sig4[4];
	char Ver4[4];
	str->Read(Sig4, 4);
	str->Read(Ver4, 4);
	this->IsV14 = false;
	if (strncmp(Sig4, "WED ", 4) == 0) {
		if (strncmp(Ver4, "V1.3", 4) == 0) {
			this->IsV14 = false;
		} else if (strncmp(Ver4, "V1.4", 4) == 0) {
			this->IsV14 = true;
		} else {
			Log(ERROR, "WEDImporter", "Unsupported WED version: {}{}{}{}", Ver4[0], Ver4[1], Ver4[2], Ver4[3]);
			return false;
		}
	} else {
		// Fallback to 8-byte check for old files
		str->Seek(0, GEM_STREAM_START);
		char Signature[8];
		str->Read(Signature, 8);
		if (strncmp(Signature, "WED V1.3", 8) != 0) {
			Log(ERROR, "WEDImporter", "This file is not a valid WED File! Actual signature: {}", Signature);
			return false;
		}
		this->IsV14 = false;
	}
	str->ReadDword(OverlaysCount);
	str->ReadDword(DoorsCount);
	str->ReadDword(OverlaysOffset);
	str->ReadDword(SecHeaderOffset);
	str->ReadDword(DoorsOffset);
	str->ReadDword(DoorTilesOffset);
	// currently unused fields from the original; likely unused completely — even commented out in wed.go implementation
	//   WORD    nVisiblityRange;
	//   WORD    nChanceOfRain; - likely unused, since it's present in the ARE file
	//   WORD    nChanceOfFog; - most likely unused, since it's present in the ARE file
	//   WORD    nChanceOfSnow; - most likely unused, since it's present in the ARE file
	//   DWORD   dwFlags;

	str->Seek(OverlaysOffset, GEM_STREAM_START);
	for (unsigned int i = 0; i < OverlaysCount; i++) {
		Overlay o;
		str->ReadSize(o.size);
		str->ReadResRef(o.TilesetResRef);
		str->ReadWord(o.UniqueTileCount);
		str->ReadWord(o.MovementType);
		str->ReadDword(o.TilemapOffset);
		str->ReadDword(o.TILOffset);
		overlays.push_back(o);
	}
	//Reading the Secondary Header
	str->Seek(SecHeaderOffset, GEM_STREAM_START);
	str->ReadDword(WallPolygonsCount);
	str->ReadDword(PolygonsOffset);
	str->ReadDword(VerticesOffset);
	str->ReadDword(WallGroupsOffset);
	str->ReadDword(PLTOffset);
	ExtendedNight = false;

	ReadWallPolygons();
	return true;
}

int WEDImporter::AddOverlay(TileMap* tm, const Overlay* newOverlays, bool rain) const
{
	int usedoverlays = 0;

	ResRef res = newOverlays->TilesetResRef;
	uint8_t len = res.length();
	// in BG1 extended night WEDs always reference the day TIS instead of the matching night TIS
	if (ExtendedNight && len == 6) {
		res[len] = 'N';
		if (!gamedata->Exists(res, IE_TIS_CLASS_ID)) {
			res[len] = '\0';
		} else {
			len++;
		}
	}
	if (rain && len < 8) {
		res[len] = 'R';
		//no rain tileset available, rolling back
		if (!gamedata->Exists(res, IE_TIS_CLASS_ID)) {
			res[len] = '\0';
		}
	}
	DataStream* tisfile = gamedata->GetResourceStream(res, IE_TIS_CLASS_ID);
	if (!tisfile) {
		return -1;
	}
	PluginHolder<TileSetMgr> tis = MakePluginHolder<TileSetMgr>(IE_TIS_CLASS_ID);
	tis->Open(tisfile);
	auto over = MakeHolder<TileOverlay>(newOverlays->size);
	for (int y = 0; y < newOverlays->size.h; y++) {
		for (int x = 0; x < newOverlays->size.w; x++) {
			// Tilemap entry size differs between versions (10 bytes in V1.3, 16 bytes in V1.4)
			size_t entrySize = this->IsV14 ? 16 : 10;
			str->Seek(newOverlays->TilemapOffset + (y * newOverlays->size.w + x) * entrySize, GEM_STREAM_START);

			ieByte overlaymask = 0;
			ieByte animspeed = ANI_DEFAULT_FRAMERATE;
			ieDword startindex32 = 0, count32 = 0, secondary32 = 0xFFFFFFFFu;
			ieWord startindex16 = 0, count16 = 0, secondary16 = 0xFFFFu;
			if (this->IsV14) {
				str->ReadDword(startindex32);
				str->ReadDword(count32);
				str->ReadDword(secondary32);
				str->Read(&overlaymask, 1); // flags
				// skip 3 unknown bytes
				ieByte skip[3];
				str->Read(skip, 3);
			} else {
				str->ReadWord(startindex16);
				str->ReadWord(count16);
				str->ReadWord(secondary16);
				str->Read(&overlaymask, 1); // bFlags in the original
				str->Read(&animspeed, 1);
				// WORD    wFlags in the original (currently unused)
				if (animspeed == 0) {
					animspeed = ANI_DEFAULT_FRAMERATE;
				}
			}

			// Read tile indices for this tilemap entry
			std::vector<ieDword> indices;
			if (this->IsV14) {
				str->Seek(newOverlays->TILOffset + startindex32 * 4, GEM_STREAM_START);
				indices.resize(count32);
				for (ieDword k = 0; k < count32; ++k) {
					ieDword idx32 = 0;
					str->ReadDword(idx32);
					indices[k] = idx32;
				}
			} else {
				str->Seek(newOverlays->TILOffset + startindex16 * 2, GEM_STREAM_START);
				indices.resize(count16);
				for (ieWord k = 0; k < count16; ++k) {
					ieWord idx16 = 0;
					str->ReadWord(idx16);
					indices[k] = static_cast<ieDword>(idx16);
				}
			}

			Tile* tile;
			if ((this->IsV14 && secondary32 == 0xFFFFFFFFu) || (!this->IsV14 && secondary16 == 0xFFFFu)) {
				tile = tis->GetTile(indices);
			} else {
				ieDword sec = 0;
				if (this->IsV14) {
					sec = secondary32;
				} else {
					sec = static_cast<ieDword>(secondary16);
				}
				tile = tis->GetTile(indices, &sec);
				// Note: if IsV14 animspeed is unknown, keep default for animation(1) too
				tile->GetAnimation(1)->fps = animspeed;
			}
			tile->GetAnimation(0)->fps = animspeed;
			tile->om = overlaymask;
			usedoverlays |= overlaymask;
			over->AddTile(std::move(*tile));
			delete tile;
		}
	}

	if (rain) {
		tm->AddRainOverlay(std::move(over));
	} else {
		tm->AddOverlay(std::move(over));
	}
	return usedoverlays;
}

//this will replace the tileset of an existing tilemap, or create a new one
TileMap* WEDImporter::GetTileMap(TileMap* tm) const
{
	int usedoverlays;
	bool freenew = false;

	if (overlays.empty()) {
		return NULL;
	}

	if (!tm) {
		tm = new TileMap();
		freenew = true;
	}

	usedoverlays = AddOverlay(tm, &overlays.at(0), false);
	if (usedoverlays == -1) {
		if (freenew) {
			delete tm;
		}
		return NULL;
	}
	// rain_overlays[0] is never used
	tm->AddRainOverlay(NULL);

	//reading additional overlays
	int mask = 2;
	for (ieDword i = 1; i < OverlaysCount; i++) {
		//skipping unused overlays
		if (!(mask & usedoverlays)) {
			tm->AddOverlay(NULL);
			tm->AddRainOverlay(NULL);
		} else {
			// FIXME: should fix AddOverlay not to load an overlay twice if there's no rain version!!
			AddOverlay(tm, &overlays.at(i), false);
			AddOverlay(tm, &overlays.at(i), true);
		}
		mask <<= 1;
	}
	return tm;
}

void WEDImporter::GetDoorPolygonCount(ieWord count, ieDword offset)
{
	ieDword basecount = offset - PolygonsOffset;
	if (basecount % WED_POLYGON_SIZE) {
		basecount += WED_POLYGON_SIZE;
		Log(WARNING, "WEDImporter", "Found broken door polygon header!");
	}
	ieDword computedIndex = basecount / WED_POLYGON_SIZE;
	Log(DEBUG, "WEDImporter",
	    "GetDoorPolygonCount: count={} offset=0x{:x} PolygonsOffset=0x{:x} basecount={} (mod {}) computedIndex={} WallPolygonsCount={}",
	    count, offset, PolygonsOffset, basecount, basecount % WED_POLYGON_SIZE, computedIndex, WallPolygonsCount);
	ieDword polycount = computedIndex + count - WallPolygonsCount;
	if (polycount > DoorPolygonsCount) {
		DoorPolygonsCount = polycount;
	}
}

WallPolygonGroup WEDImporter::OpenDoorPolygons() const
{
	size_t index = (OpenPolyOffset - PolygonsOffset) / WED_POLYGON_SIZE;
	size_t count = OpenPolyCount;
	return MakeGroupFromTableEntries(index, count);
}

WallPolygonGroup WEDImporter::ClosedDoorPolygons() const
{
	// Door polygon offsets may point to fields within polygon structures, not polygon starts
	// Calculate the polygon index by finding which polygon contains this offset
	const size_t relativeOffset = ClosedPolyOffset - PolygonsOffset;
	const size_t index = relativeOffset / WED_POLYGON_SIZE;
	const size_t count = ClosedPolyCount;
	Log(DEBUG, "WEDImporter",
	    "ClosedDoorPolygons: PolygonsOffset=0x{:x} ClosedPolyOffset=0x{:x} relativeOffset={} index={} count={} polygonTable.size={}",
	    PolygonsOffset, ClosedPolyOffset, relativeOffset, index, count, polygonTable.size());
	return MakeGroupFromTableEntries(index, count);
}

std::vector<ieDword> WEDImporter::GetDoorIndices(const ResRef& resref, bool& BaseClosed)
{
	ieWord DoorClosed;
	ieDword DoorTileStart32 = 0, DoorTileCount32 = 0;
	ieWord DoorTileStart16 = 0, DoorTileCount16 = 0;
	ResRef Name;
	unsigned int i;

	for (i = 0; i < DoorsCount; i++) {
		const ieDword doorSize = this->IsV14 ? 30 : 0x1A;
		str->Seek(DoorsOffset + (i * doorSize), GEM_STREAM_START);
		str->ReadResRef(Name);
		if (Name == resref)
			break;
	}
	//The door has no representation in the WED file
	if (i == DoorsCount) {
		Log(ERROR, "WEDImporter", "Found door without WED entry!");
		return {};
	}

	str->ReadWord(DoorClosed);
	if (IsV14) {
		str->ReadDword(DoorTileStart32);
		str->ReadDword(DoorTileCount32);
		str->ReadWord(OpenPolyCount);
		str->ReadWord(ClosedPolyCount);
		str->ReadDword(OpenPolyOffset);
		str->ReadDword(ClosedPolyOffset);
	} else {
		str->ReadWord(DoorTileStart16);
		str->ReadWord(DoorTileCount16);
		str->ReadWord(OpenPolyCount);
		str->ReadWord(ClosedPolyCount);
		str->ReadDword(OpenPolyOffset);
		str->ReadDword(ClosedPolyOffset);
	}

	//Reading Door Tile Cells
	std::vector<ieDword> DoorTiles;
	if (this->IsV14) {
		str->Seek(DoorTilesOffset + (DoorTileStart32 * 4), GEM_STREAM_START);
		DoorTiles.resize(DoorTileCount32);
		for (ieDword k = 0; k < DoorTileCount32; ++k) {
			ieDword idx32 = 0;
			str->ReadDword(idx32);
			DoorTiles[k] = idx32;
		}
	} else {
		str->Seek(DoorTilesOffset + (DoorTileStart16 * 2), GEM_STREAM_START);
		DoorTiles.resize(DoorTileCount16);
		for (ieWord k = 0; k < DoorTileCount16; ++k) {
			ieWord idx16 = 0;
			str->ReadWord(idx16);
			DoorTiles[k] = static_cast<ieDword>(idx16);
		}
	}

	BaseClosed = DoorClosed != 0;
	return DoorTiles;
}

void WEDImporter::ReadWallPolygons()
{
	// Determine the highest polygon index referenced by any door (open/closed)
	ieDword doorMaxIndex = 0;
	bool haveDoorPolys = false;

	for (ieDword i = 0; i < DoorsCount; i++) {
		const ieDword doorSize = this->IsV14 ? 30 : 0x1A;
		const ieDword polyOffset = this->IsV14 ? 18 : 14; // after ResRef(8)+Closed(2)+Start+Count
		str->Seek(DoorsOffset + (i * doorSize) + polyOffset, GEM_STREAM_START);

		str->ReadWord(OpenPolyCount);
		str->ReadWord(ClosedPolyCount);
		str->ReadDword(OpenPolyOffset);
		str->ReadDword(ClosedPolyOffset);


		// Compute referenced polygon indices for this door; offsets may point inside a polygon
		if (OpenPolyCount) {
			const ieDword rel = OpenPolyOffset - PolygonsOffset;
			const ieDword firstIdx = rel / WED_POLYGON_SIZE;
			const ieDword lastIdx = firstIdx + (OpenPolyCount - 1);
			doorMaxIndex = std::max(doorMaxIndex, lastIdx);
			haveDoorPolys = true;
		}
		if (ClosedPolyCount) {
			const ieDword rel = ClosedPolyOffset - PolygonsOffset;
			const ieDword firstIdx = rel / WED_POLYGON_SIZE;
			const ieDword lastIdx = firstIdx + (ClosedPolyCount - 1);
			doorMaxIndex = std::max(doorMaxIndex, lastIdx);
			haveDoorPolys = true;
		}
	}

	// Compute final polygon count robustly
	ieDword polygonCount = WallPolygonsCount;
	if (haveDoorPolys) {
		polygonCount = std::max(polygonCount, doorMaxIndex + 1);
	}
	Log(DEBUG, "WEDImporter",
	    "ReadWallPolygons: WallPolygonsCount={} doorMaxIndex={} -> polygonCount={} PolygonsOffset=0x{:x} VerticesOffset=0x{:x}",
	    WallPolygonsCount, doorMaxIndex, polygonCount, PolygonsOffset, VerticesOffset);

	struct wed_polygon {
		ieDword FirstVertex;
		ieDword CountVertex;
		ieByte Flags;
		ieByte Height; // typically set to -1, unsure if used
		Region rect;
	};

	polygonTable.resize(polygonCount);
	wed_polygon* PolygonHeaders = new wed_polygon[polygonCount];

	str->Seek(PolygonsOffset, GEM_STREAM_START);

	for (ieDword i = 0; i < polygonCount; i++) {
		str->ReadDword(PolygonHeaders[i].FirstVertex);
		str->ReadDword(PolygonHeaders[i].CountVertex);
		str->Read(&PolygonHeaders[i].Flags, 1);
		str->Read(&PolygonHeaders[i].Height, 1);

		// Note: unlike the rest, the layout is minX, maxX, minY, maxY
		auto& rect = PolygonHeaders[i].rect;
		str->ReadScalar<int, ieWord>(rect.x);
		str->ReadScalar<int, ieWord>(rect.w);
		str->ReadScalar<int, ieWord>(rect.y);
		str->ReadScalar<int, ieWord>(rect.h);

		rect.w -= rect.x;
		rect.h -= rect.y;
	}

	for (ieDword i = 0; i < polygonCount; i++) {
		str->Seek(PolygonHeaders[i].FirstVertex * 4 + VerticesOffset, GEM_STREAM_START);
		//compose polygon
		ieDword count = PolygonHeaders[i].CountVertex;
		if (count < 3) {
			//danger, danger
			continue;
		}
		ieDword flags = PolygonHeaders[i].Flags & ~(WF_BASELINE | WF_HOVER);
		Point base0, base1;
		if (PolygonHeaders[i].Flags & WF_HOVER) {
			count -= 2;
			str->ReadPoint(base0);
			str->ReadPoint(base1);
			flags |= WF_BASELINE;
		}
		std::vector<Point> points(count);
		for (size_t j = 0; j < count; ++j) {
			Point vertex;
			str->ReadPoint(vertex);
			points[j] = vertex;
		}

		if (!(flags & WF_BASELINE)) {
			if (PolygonHeaders[i].Flags & WF_BASELINE) {
				base0 = points[0];
				base1 = points[1];
				flags |= WF_BASELINE;
			}
		}

		const Region& rgn = PolygonHeaders[i].rect;
		if (!rgn.size.IsInvalid()) { // PST AR0600 is known to have a polygon with 0 height
			polygonTable[i] = std::make_shared<WallPolygon>(std::move(points), &rgn);
			if (flags & WF_BASELINE) {
				polygonTable[i]->SetBaseline(base0, base1);
			}
			if (core->HasFeature(GFFlags::PST_STATE_FLAGS)) {
				flags |= WF_COVERANIMS;
			}
			polygonTable[i]->SetPolygonFlag(flags);
		}
	}
	delete[] PolygonHeaders;
}

WallPolygonGroup WEDImporter::MakeGroupFromTableEntries(size_t idx, size_t cnt) const
{
	Log(DEBUG, "WEDImporter", "MakeGroupFromTableEntries: idx={} cnt={} polygonTable.size={}", idx, cnt, polygonTable.size());
	auto begin = polygonTable.begin() + idx;
	auto end = begin + cnt;
	WallPolygonGroup grp;
	std::copy_if(begin, end, std::back_inserter(grp), [](const std::shared_ptr<WallPolygon>& wp) {
		return wp != nullptr;
	});
	return grp;
}

std::vector<WallPolygonGroup> WEDImporter::GetWallGroups() const
{
	str->Seek(PLTOffset, GEM_STREAM_START);
	size_t PLTSize = (VerticesOffset > PLTOffset ? VerticesOffset - PLTOffset : PLTOffset - VerticesOffset) / 2;
	std::vector<ieWord> PLT(PLTSize);

	for (ieWord& idx : PLT) {
		str->ReadWord(idx);
	}

	Log(DEBUG, "WEDImporter", "GetWallGroups: PLTOffset=0x{:x} VerticesOffset=0x{:x} PLTSize={} polygonTable.size={}",
	    PLTOffset, VerticesOffset, PLTSize, polygonTable.size());

	auto ceilInt = [](int32_t v, int32_t div) {
		if (v % div == 0) {
			return v / div;
		}

		return (v + (div - (v % div))) / div;
	};

	// was error-prone w.r.t. IEEE754 optimization: ceilf(overlays[0].size.w / 10.0f) * ceilf(overlays[0].size.h / 7.5f)
	auto w = overlays[0].size.w;
	auto h = overlays[0].size.h * 2;
	size_t groupSize = ceilInt(w, 10) * ceilInt(h, 15);

	std::vector<WallPolygonGroup> polygonGroups;
	polygonGroups.reserve(groupSize);

	// The wallgroups table ends at PLTOffset. Each entry is 2x WORD (index,count) => 4 bytes.
	const size_t actualGroups = (PLTOffset > WallGroupsOffset) ? (PLTOffset - WallGroupsOffset) / 4 : 0;
	Log(DEBUG, "WEDImporter", "GetWallGroups: computedGroupSize={} actualGroups={} (bytes: {})",
	    groupSize, actualGroups, (PLTOffset > WallGroupsOffset) ? (PLTOffset - WallGroupsOffset) : 0);
	if (actualGroups && actualGroups != groupSize) {
		Log(WARNING, "WEDImporter", "Wall group count mismatch: computed={} file={} — capping to file size to avoid overread.",
		    groupSize, actualGroups);
	}

	const size_t loopCount = (actualGroups && actualGroups < groupSize) ? actualGroups : groupSize;
	str->Seek(WallGroupsOffset, GEM_STREAM_START);
	for (size_t i = 0; i < loopCount; ++i) {
		ieWord index16 = 0, count16 = 0;
		str->ReadWord(index16);
		str->ReadWord(count16);
		const size_t index = static_cast<size_t>(index16);
		const size_t count = static_cast<size_t>(count16);

		polygonGroups.emplace_back();
		WallPolygonGroup& group = polygonGroups.back();
		Log(DEBUG, "WEDImporter", "WallGroup {}: startIndex={} count={} (PLT range [{} , {}))", i, index, count, index, index + count);

		for (size_t j = index; j < index + count; ++j) {
			ieWord polyIndex = PLT[j];
			Log(DEBUG, "WEDImporter", "  PLT[{}] -> polyIndex={} polygonTable.size={}", j, polyIndex, polygonTable.size());
			auto wp = polygonTable[polyIndex];
			if (wp) {
				group.push_back(wp);
			}
		}
	}

	return polygonGroups;
}

#include "plugindef.h"

GEMRB_PLUGIN(0x7486BE7, "WED File Importer")
PLUGIN_CLASS(IE_WED_CLASS_ID, WEDImporter)
END_PLUGIN()
