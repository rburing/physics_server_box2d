#include "box2d_direct_space_state.h"
#include "b2_user_settings.h"
#include "box2d_collision_object.h"
#include "box2d_type_conversions.h"

#include <box2d/b2_fixture.h>

class IntersectRayCastCallback : public b2RayCastCallback {
public:
	PhysicsServer2DExtensionRayResult *result;
	uint32_t collision_mask;
	bool collide_with_bodies;
	bool collide_with_areas;
	bool hit_from_inside;
	bool hit = false;

	virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
			const b2Vec2 &normal, float fraction) override {
		if (fixture->GetFilterData().maskBits | collision_mask &&
				(fixture->IsSensor() && collide_with_areas ||
						!fixture->IsSensor() && collide_with_bodies)) {
			result->normal = box2d_to_godot(normal);
			result->position = box2d_to_godot(point);
			result->shape = fixture->GetUserData().shape_idx;
			Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
			result->rid = collision_object->get_self();
			result->collider_id = collision_object->get_object_instance_id();
			result->collider = collision_object->get_object();
			hit = true;
			return 0;
		}
		return 1;
	}
};

PhysicsDirectSpaceState2D *Box2DDirectSpaceState::get_space_state() {
	ERR_FAIL_NULL_V(space, nullptr);
	return space->get_direct_state();
}

bool Box2DDirectSpaceState::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *result) {
	IntersectRayCastCallback callback;
	callback.result = result;
	callback.collide_with_bodies = collide_with_bodies;
	callback.collide_with_areas = collide_with_areas;
	callback.hit_from_inside = hit_from_inside;
	space->get_b2World()->RayCast(&callback, godot_to_box2d(from), godot_to_box2d(to));
	return callback.hit;
}
int32_t Box2DDirectSpaceState::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *results, int32_t max_results) {
	return 0;
}
int32_t Box2DDirectSpaceState::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *result, int32_t max_results) {
	return 0;
}
bool Box2DDirectSpaceState::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *closest_safe, float *closest_unsafe) {
	return false;
}
bool Box2DDirectSpaceState::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	return false;
}
bool Box2DDirectSpaceState::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *rest_info) {
	return false;
}
