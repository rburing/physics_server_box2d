#include "box2d_query_callback.h"

#include "../b2_user_settings.h"

Box2DQueryCallback::Box2DQueryCallback(uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas) {
	collision_mask = p_collision_mask;
	collide_with_bodies = p_collide_with_bodies;
	collide_with_areas = p_collide_with_areas;
}

Vector<b2Fixture *> Box2DQueryCallback::get_results() {
	return results;
}

bool Box2DQueryCallback::ReportFixture(b2Fixture *fixture) {
	if ( // collision mask
			((fixture->GetFilterData().maskBits & collision_mask) != 0) &&
			// collide with area or body bit
			((fixture->IsSensor() && collide_with_areas) ||
					(!fixture->IsSensor() && collide_with_bodies))) {
		results.append(fixture);
	}
	return true;
}
