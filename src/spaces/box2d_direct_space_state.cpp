#include "box2d_direct_space_state.h"

#include "../b2_user_settings.h"

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "../servers/box2d_sweep_test.h"
#include "../servers/physics_server_box2d.h"
#include "box2d_query_callback.h"
#include "box2d_query_point_callback.h"
#include "box2d_ray_cast_callback.h"

#include <box2d/b2_collision.h>
#include <box2d/b2_fixture.h>

PhysicsDirectSpaceState2D *Box2DDirectSpaceState::get_space_state() {
	ERR_FAIL_NULL_V(space, nullptr);
	return space->get_direct_state();
}

bool Box2DDirectSpaceState::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *result) {
	// TODO take into account hit_from_inside
	Box2DRayCastCallback callback(result, collision_mask, collide_with_bodies, collide_with_areas, hit_from_inside);
	space->get_b2World()->RayCast(&callback, godot_to_box2d(from), godot_to_box2d(to));
	return callback.get_hit();
}
int32_t Box2DDirectSpaceState::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *results, int32_t max_results) {
	Box2DQueryPointCallback callback(collision_mask,
			collide_with_bodies,
			collide_with_areas,
			canvas_instance_id,
			godot_to_box2d(position),
			max_results);
	b2Vec2 pos(godot_to_box2d(position));
	b2AABB aabb;
	float point_size = 0.000000001f;

	aabb.lowerBound.Set(pos.x - point_size, pos.y - point_size);
	aabb.upperBound.Set(pos.x + point_size, pos.y + point_size);
	space->get_b2World()->QueryAABB(&callback, aabb);
	Vector<b2Fixture *> collision_results = callback.get_results();
	for (b2Fixture *fixture : collision_results) {
		Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
		PhysicsServer2DExtensionShapeResult &result = *results++;

		result.shape = fixture->GetUserData().shape_idx;
		result.rid = collision_object->get_self();
		result.collider_id = collision_object->get_object_instance_id();
		result.collider = collision_object->get_object_unsafe();
	}
	return collision_results.size();
}
b2AABB get_shape_aabb(const Box2DShape *shape, const b2Transform &shape_transform) {
	b2AABB aabb;
	b2AABB aabb_total;
	bool first_time = true;
	for (b2Shape *b2_shape : shape->get_b2_shapes()) {
		b2_shape->ComputeAABB(&aabb, shape_transform, 0);
		if (first_time) {
			first_time = false;
			aabb_total = aabb;
		} else {
			aabb_total.Combine(aabb);
		}
	}
	return aabb_total;
}
int32_t Box2DDirectSpaceState::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *result, int32_t max_results) {
	// TODO margin unused here
	if (max_results == 0) {
		return 0;
	}
	const Box2DShape *shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!shape, 0);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	int count = 0;
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		for (b2Shape *shape_A : shape->get_b2_shapes()) {
			b2DistanceOutput output = Box2DSweepTest::call_b2_distance(shape_transform, shape_A, fixture_B->GetBody()->GetTransform(), shape_B);
			if (output.distance < 10.0f * b2_epsilon) {
				PhysicsServer2DExtensionShapeResult &result_instance = result[count++];
				result_instance.shape = fixture_B->GetUserData().shape_idx;
				result_instance.rid = fixture_B->GetUserData().shape->get_self();
				result_instance.collider_id = fixture_B->GetUserData().shape->get_body()->get_object_instance_id();
				result_instance.collider = fixture_B->GetUserData().shape->get_body()->get_object_unsafe();
				if (count >= max_results) {
					return count;
				}
			}
		}
	}
	return count;
}
bool Box2DDirectSpaceState::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *closest_safe, float *closest_unsafe) {
	// TODO margin unused here?
	const Box2DShape *shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!shape, 0);
	b2Transform shape_A_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(collision_mask,
			collide_with_bodies,
			collide_with_areas);
	b2AABB aabb = get_shape_aabb(shape, shape_A_transform);
	aabb.Combine(get_shape_aabb(shape, b2Transform(godot_to_box2d(motion), b2Rot())));
	space->get_b2World()->QueryAABB(&callback, aabb);
	Vector<b2Fixture *> collision_results = callback.get_results();
	b2Sweep sweepA = Box2DSweepTest::create_b2_sweep(shape_A_transform, shape->get_body()->get_b2Body()->GetLocalCenter(), godot_to_box2d(motion));
	float closest_dist = -1;
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		b2Body *body_B = fixture_B->GetBody();
		b2Sweep sweepB = Box2DSweepTest::create_b2_sweep(body_B->GetTransform(), body_B->GetLocalCenter(), b2Vec2_zero);
		for (b2Shape *shape_A : shape->get_b2_shapes()) {
			Box2DSweepTest::Box2DSweepTestResult output = Box2DSweepTest::shape_cast(shape_A_transform, shape_A, sweepA, body_B->GetTransform(), shape_B, sweepB, space->get_step());
			if (output.intersects) {
				if (closest_dist < 0) {
					closest_dist = (output.place_of_impact.p + output.output.pointA - shape_A_transform.p).Length();
				} else {
					closest_dist = (output.place_of_impact.p + output.output.pointA - shape_A_transform.p).Length();
				}
				return true;
			}
		}
	}
	*closest_unsafe = closest_dist;
	*closest_safe = closest_dist - margin;
	return false;
}
bool Box2DDirectSpaceState::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	// TODO margin unused here
	if (max_results == 0) {
		return false;
	}
	const Box2DShape *shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!shape, 0);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	auto *result = static_cast<Vector2 *>(results);
	*result_count = 0;
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		for (b2Shape *shape_A : shape->get_b2_shapes()) {
			b2DistanceOutput output = Box2DSweepTest::call_b2_distance(shape_transform, shape_A, fixture_B->GetBody()->GetTransform(), shape_B);
			if (output.distance < 10.0f * b2_epsilon) {
				result[*result_count++] = box2d_to_godot(output.pointA);
				if (*result_count >= max_results) {
					return true;
				}
			}
		}
	}
	return *result_count != 0;
}
bool Box2DDirectSpaceState::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *rest_info) {
	// TODO margin unused here
	const Box2DShape *shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!shape, 0);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		for (b2Shape *shape_A : shape->get_b2_shapes()) {
			b2DistanceOutput output = Box2DSweepTest::call_b2_distance(shape_transform, shape_A, fixture_B->GetBody()->GetTransform(), shape_B);
			if (output.distance < 10.0f * b2_epsilon) {
				PhysicsServer2DExtensionShapeRestInfo &result_instance = *rest_info++;
				result_instance.shape = fixture_B->GetUserData().shape_idx;
				result_instance.rid = fixture_B->GetUserData().shape->get_self();
				result_instance.collider_id = fixture_B->GetUserData().shape->get_body()->get_object_instance_id();
				result_instance.point = box2d_to_godot(output.pointB);
				result_instance.normal = box2d_to_godot(output.pointB - output.pointB).normalized();
				result_instance.linear_velocity = box2d_to_godot(fixture_B->GetBody()->GetLinearVelocity());
			}
		}
	}
	return collision_results.size();
}
