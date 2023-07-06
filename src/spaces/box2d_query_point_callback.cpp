#include "box2d_query_point_callback.h"

#include "../b2_user_settings.h"

Box2DQueryPointCallback::Box2DQueryPointCallback(uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		uint64_t p_canvas_instance_id,
		b2Vec2 p_position,
		int32_t p_max_results) {
	collision_mask = p_collision_mask;
	collide_with_bodies = p_collide_with_bodies;
	collide_with_areas = p_collide_with_areas;
	canvas_instance_id = p_canvas_instance_id;
	position = p_position;
	max_results = p_max_results;
}

Vector<b2Fixture *> Box2DQueryPointCallback::get_results() {
	return results;
}

bool Box2DQueryPointCallback::ReportFixture(b2Fixture *fixture) {
	if ( // collision mask
			((fixture->GetFilterData().maskBits & collision_mask) != 0) &&
			// collide with area or body bit
			((fixture->IsSensor() && collide_with_areas) ||
					(!fixture->IsSensor() && collide_with_bodies)) &&
			// test for point containment
			(fixture->GetShape()->TestPoint(fixture->GetBody()->GetTransform(), position))) {
		hit_count++;

		Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
		if (collision_object->get_canvas_instance_id() != godot::ObjectID(canvas_instance_id)) {
			return false;
		}
		results.append(fixture);
	}
	return max_results - hit_count > 0;
}
