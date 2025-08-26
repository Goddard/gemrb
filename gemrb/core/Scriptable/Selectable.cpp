/* GemRB - Infinity Engine Emulator
 * Copyright (C) 2024 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#include "Selectable.h"

#include "Interface.h"

#include "GUI/GUIAnimation.h"
#include "Video/Video.h"

namespace GemRB {

void Selectable::SetBBox(const Region& newBBox)
{
	BBox = newBBox;
}

// NOTE: still need to multiply by 4 or 3 to get full pixel radii
int Selectable::CircleSize2Radius(int circleSize)
{
	// for size >= 2, radii are (size-1)*16, (size-1)*12
	// for size == 1, radii are 12, 9
	int adjustedSize = (circleSize - 1) * 4;
	if (adjustedSize < 4) adjustedSize = 3;
	return adjustedSize;
}

int Selectable::CircleSize2Radius() const
{
	return CircleSize2Radius(this->circleSize);
}

void Selectable::DrawCircle(const Point& p) const
{
	if (circleSize <= 0) {
		return;
	}

	Color mix;
	const Color* col = &selectedColor;
	Holder<Sprite2D> sprite = circleBitmap[0];

	if (Selected && !Over) {
		sprite = circleBitmap[1];
	} else if (Over) {
		mix = GlobalColorCycle.Blend(overColor, selectedColor);
		col = &mix;
	} else if (IsPC()) {
		// only dim base EA colors
		if (*col == ColorGreen || *col == ColorBlue || *col == ColorRed) col = &overColor;
	}

	if (sprite) {
		// Use game-world blit (tile-aligned) so circle matches actor/tiles at zoom
		VideoDriver->BlitGameSprite(sprite, Pos - p, BlitFlags::BLENDED);
	} else {
		// Draw ellipse fallback, but bypass BasePoint scaling to avoid drift at zoom
		const int ups = core->config.UpScaleFactor;
		auto baseSize = (int) (CircleSize2Radius() * sizeFactor * ups);
		const Size unscaledSize(baseSize * 8, baseSize * 6);
		Region rUnscaled(Pos - p - unscaledSize.Center(), unscaledSize);

		float gs = VideoDriver->GetGameScale();
		if (gs != 1.0f) {
			const double s = (double) gs;
			const int left = (int) std::floor(((double) rUnscaled.x) * s);
			const int top = (int) std::floor(((double) rUnscaled.y) * s);
			const int right = (int) std::floor(((double) (rUnscaled.x + rUnscaled.w)) * s);
			const int bottom = (int) std::floor(((double) (rUnscaled.y + rUnscaled.h)) * s);
			Region rScaled(left, top, right - left, bottom - top);
			VideoDriver->SetGameScale(1.0f);
			VideoDriver->DrawEllipse(rScaled, *col);
			VideoDriver->SetGameScale(gs);
		} else {
			VideoDriver->DrawEllipse(rUnscaled, *col);
		}
	}
}

// Check if P is over our ground circle
bool Selectable::IsOver(const Point& P) const
{
	return IsOver(P, Pos);
}

bool Selectable::IsOver(const Point& P, const Point& CenterPos) const
{
	// Mirror DrawCircle(): decide which sprite would be used visually
	Holder<Sprite2D> sprite = circleBitmap[0];
	if (Selected && !Over) {
		sprite = circleBitmap[1];
	}

	if (sprite) {
		// Rectangle hit-test based on the actual sprite frame size, centered on CenterPos
		const int halfW = sprite->Frame.w / 2;
		const int halfH = sprite->Frame.h / 2;
		const Point d = P - CenterPos;
		if (d.x < -halfW || d.x > halfW) return false;
		if (d.y < -halfH || d.y > halfH) return false;
		return true;
	}

	// Ellipse hit-test mirroring the ellipse we draw in DrawCircle()
	// base radii are derived from CircleSize2Radius(), sizeFactor and UpScaleFactor
	const int ups = core->config.UpScaleFactor;
	const int baseSize = (int) (CircleSize2Radius() * sizeFactor) * ups;
	const int rx = (baseSize * 8) / 2; // half width
	const int ry = (baseSize * 6) / 2; // half height
	const Point d = P - CenterPos;

	// Avoid floating point: test (dx^2 / rx^2) + (dy^2 / ry^2) <= 1
	const long long dx = d.x;
	const long long dy = d.y;
	const long long rx2 = (long long) rx * rx;
	const long long ry2 = (long long) ry * ry;
	const long long lhs = dx * dx * ry2 + dy * dy * rx2;
	const long long rhs = rx2 * ry2;
	return lhs <= rhs;
}

bool Selectable::IsSelected() const
{
	return Selected == 1;
}

void Selectable::SetOver(bool over)
{
	Over = over;
}

//don't call this function after rendering the cover and before the
//blitting of the sprite or bad things will happen :)
void Selectable::Select(int Value)
{
	if (Selected != 0x80 || Value != 1) {
		Selected = (ieWord) Value;
	}
}

void Selectable::SetCircle(int circlesize, float_t factor, const Color& color, Holder<Sprite2D> normal_circle, Holder<Sprite2D> selected_circle)
{
	circleSize = circlesize;
	sizeFactor = factor;
	selectedColor = color;
	overColor.r = color.r >> 1;
	overColor.g = color.g >> 1;
	overColor.b = color.b >> 1;
	overColor.a = color.a;
	circleBitmap[0] = std::move(normal_circle);
	circleBitmap[1] = std::move(selected_circle);
}

}
