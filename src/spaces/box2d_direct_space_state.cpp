#include "box2d_direct_space_state.h"

#include "../b2_user_settings.h"

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "../servers/physics_server_box2d.h"
#include "box2d_query_callback.h"
#include "box2d_query_point_callback.h"
#include "box2d_ray_cast_callback.h"

#include <box2d/b2_collision.h>
#include <box2d/b2_fixture.h>

#define POINT_SIZE 0.000000001f

PhysicsDirectSpaceState2D *Box2DDirectSpaceState::get_space_state() {
	ERR_FAIL_NULL_V(space, nullptr);
	return space->get_direct_state();
}

bool Box2DDirectSpaceState::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *result) {
	Box2DRayCastCallback callback(this, result, collision_mask, collide_with_bodies, collide_with_areas, hit_from_inside);
	space->get_b2World()->RayCast(&callback, godot_to_box2d(from), godot_to_box2d(to));
	if (callback.get_hit()) {
		return true;
	}
	// also try point intersect
	if (hit_from_inside) {
		// try raycasting from other direction
		Box2DRayCastCallback callback_other_dir(this, result, collision_mask, collide_with_bodies, collide_with_areas, hit_from_inside);
		space->get_b2World()->RayCast(&callback_other_dir, godot_to_box2d(to), godot_to_box2d(from));
		if (callback_other_dir.get_hit()) {
			return true;
		}
		// try only a point in case the ray is completely inside
		Box2DQueryPointCallback callback(this,
				collision_mask,
				collide_with_bodies,
				collide_with_areas,
				-1,
				false,
				godot_to_box2d(from),
				10);
		b2Vec2 pos(godot_to_box2d(from));
		b2AABB aabb;

		aabb.lowerBound.Set(pos.x - POINT_SIZE, pos.y - POINT_SIZE);
		aabb.upperBound.Set(pos.x + POINT_SIZE, pos.y + POINT_SIZE);
		space->get_b2World()->QueryAABB(&callback, aabb);
		ERR_PRINT(itos(callback.get_results().size()));
		if (callback.get_results().size() != 0) {
			b2Fixture *fixture = callback.get_results()[0];
			result->normal = (box2d_to_godot(fixture->GetBody()->GetPosition()) - from).normalized();
			result->position = box2d_to_godot(fixture->GetBody()->GetPosition());
			result->shape = fixture->GetUserData().shape_idx;
			Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
			result->rid = collision_object->get_self();
			result->collider_id = collision_object->get_object_instance_id();
			result->collider = collision_object->get_object_unsafe();
			return true;
		}
	}
	return false;
}
int32_t Box2DDirectSpaceState::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *results, int32_t max_results) {
	if (max_results == 0) {
		return 0;
	}
	Box2DQueryPointCallback callback(this,
			collision_mask,
			collide_with_bodies,
			collide_with_areas,
			canvas_instance_id,
			true,
			godot_to_box2d(position),
			max_results);
	b2Vec2 pos(godot_to_box2d(position));
	b2AABB aabb;

	aabb.lowerBound.Set(pos.x - POINT_SIZE, pos.y - POINT_SIZE);
	aabb.upperBound.Set(pos.x + POINT_SIZE, pos.y + POINT_SIZE);
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
b2AABB get_shape_aabb(Box2DShape *shape, const b2Transform &shape_transform) {
	b2AABB aabb;
	b2AABB aabb_total;
	bool first_time = true;
	for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
		b2Shape *b2_shape = (shape->get_transformed_b2Shape(i, godot::Transform2D(), true, false));
		b2_shape->ComputeAABB(&aabb, shape_transform, 0);
		if (first_time) {
			first_time = false;
			aabb_total = aabb;
		} else {
			aabb_total.Combine(aabb);
		}
		memdelete(b2_shape);
	}
	ERR_PRINT(itos(aabb.lowerBound.x) + " " + itos(aabb.lowerBound.y));
	ERR_PRINT(itos(aabb.upperBound.x) + " " + itos(aabb.upperBound.y));
	return aabb_total;
}
int32_t Box2DDirectSpaceState::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *result, int32_t max_results) {
	ERR_PRINT("called intersect shape");
	// TODO margin unused here
	if (max_results == 0) {
		return 0;
	}
	const Box2DShape *const_shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!const_shape, 0);
	Box2DShape *shape = const_cast<Box2DShape *>(const_shape);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(this,
			collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	int count = 0;
	ERR_PRINT(itos(collision_results.size()));
	for (b2Fixture *fixture_B : collision_results) {
		if (count >= max_results) {
			break;
		}
		b2Shape *shape_B = fixture_B->GetShape();
		for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
			if (count >= max_results) {
				break;
			}
			b2Shape *shape_A = shape->get_transformed_b2Shape(i, Transform2D(), false, false);
			b2DistanceOutput output = Box2DSweepTest::call_b2_distance(shape_transform, shape_A, fixture_B->GetBody()->GetTransform(), shape_B);
			if (output.distance < 10.0f * b2_epsilon) {
				PhysicsServer2DExtensionShapeResult &result_instance = result[count++];
				result_instance.shape = fixture_B->GetUserData().shape_idx;
				result_instance.rid = fixture_B->GetUserData().shape->get_self();
				result_instance.collider_id = fixture_B->GetUserData().shape->get_body()->get_object_instance_id();
				result_instance.collider = fixture_B->GetUserData().shape->get_body()->get_object_unsafe();
			}
			memdelete(shape_A);
		}
	}
	return count;
}
Box2DSweepTest::Box2DSweepTestResult Box2DDirectSpaceState::_test_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas) {
	ERR_PRINT("called test motion");
	// TODO margin unused here?
	const Box2DShape *const_shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!const_shape, Box2DSweepTest::Box2DSweepTestResult());
	Box2DShape *shape = const_cast<Box2DShape *>(const_shape);
	b2Transform shape_A_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(this,
			collision_mask,
			collide_with_bodies,
			collide_with_areas);
	b2AABB aabb = get_shape_aabb(shape, shape_A_transform);
	aabb.Combine(get_shape_aabb(shape, b2Transform(godot_to_box2d(motion), b2Rot())));
	space->get_b2World()->QueryAABB(&callback, aabb);
	Vector<b2Fixture *> collision_results = callback.get_results();
	b2Sweep sweepA = Box2DSweepTest::create_b2_sweep(shape_A_transform, shape->get_body()->get_b2Body()->GetLocalCenter(), godot_to_box2d(motion));
	Box2DSweepTest::Box2DSweepTestResult result;
	result.intersects = false;
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		b2Body *body_B = fixture_B->GetBody();
		b2Sweep sweepB = Box2DSweepTest::create_b2_sweep(body_B->GetTransform(), body_B->GetLocalCenter(), b2Vec2_zero);
		for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
			b2Shape *shape_A = shape->get_transformed_b2Shape(i, Transform2D(), false, false);
			Box2DSweepTest::Box2DSweepTestResult output = Box2DSweepTest::shape_cast(shape_A_transform, shape_A, shape->get_body()->get_b2Body(), sweepA, body_B->GetTransform(), shape_B, body_B, sweepB, space->get_step());

			if (output.intersects) {
				if (!result.intersects || result.distance > output.distance) {
					result = output;
				}
			}
			memdelete(shape_A);
		}
	}
	return result;
}
bool Box2DDirectSpaceState::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *closest_safe, float *closest_unsafe) {
	ERR_PRINT("called cast motion");
	// TODO margin unused here?
	const Box2DShape *shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!shape, 0);
	b2Transform shape_A_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));

	Box2DSweepTest::Box2DSweepTestResult result = _test_motion(shape_rid, transform, motion, margin, collision_mask, collide_with_bodies, collide_with_areas);
	*closest_unsafe = result.distance;
	*closest_safe = result.distance - margin;
	return result.intersects;
}
bool Box2DDirectSpaceState::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	ERR_PRINT("called collide shape");
	// TODO margin unused here
	if (max_results == 0) {
		return false;
	}
	const Box2DShape *const_shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!const_shape, 0);
	Box2DShape *shape = const_cast<Box2DShape *>(const_shape);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(this,
			collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	auto *result = static_cast<Vector2 *>(results);
	*result_count = 0;
	for (b2Fixture *fixture_B : collision_results) {
		if (*result_count >= max_results) {
			break;
		}
		b2Shape *shape_B = fixture_B->GetShape();
		for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
			if (*result_count >= max_results) {
				break;
			}
			b2Shape *shape_A = shape->get_transformed_b2Shape(i, Transform2D(), false, false);
			b2DistanceOutput output = Box2DSweepTest::call_b2_distance(shape_transform, shape_A, fixture_B->GetBody()->GetTransform(), shape_B);
			if (output.distance < 10.0f * b2_epsilon) {
				result[*result_count++] = box2d_to_godot(output.pointA);
			}
			memdelete(shape_A);
		}
	}
	return *result_count != 0;
}
bool Box2DDirectSpaceState::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *rest_info) {
	ERR_PRINT("called rest info");
	// TODO margin unused here
	const Box2DShape *const_shape = space->get_server()->shape_owner.get_or_null(shape_rid);
	ERR_FAIL_COND_V(!const_shape, 0);
	Box2DShape *shape = const_cast<Box2DShape *>(const_shape);
	b2Transform shape_transform(godot_to_box2d(transform.get_origin()), b2Rot(transform.get_rotation()));
	Box2DQueryCallback callback(this,
			collision_mask,
			collide_with_bodies,
			collide_with_areas);
	space->get_b2World()->QueryAABB(&callback, get_shape_aabb(shape, shape_transform));
	Vector<b2Fixture *> collision_results = callback.get_results();
	for (b2Fixture *fixture_B : collision_results) {
		b2Shape *shape_B = fixture_B->GetShape();
		for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
			b2Shape *shape_A = shape->get_transformed_b2Shape(i, Transform2D(), false, false);
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
			memdelete(shape_A);
		}
	}
	return collision_results.size();
}
