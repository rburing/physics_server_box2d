#pragma once

#include <box2d/b2_distance.h>
#include <godot_cpp/templates/vector.hpp>
using namespace godot;

class b2Vec2;
class b2Transform;
class b2TOIOutput;
class b2Body;
class b2Shape;
class b2Sweep;

class Box2DSweepTest {
public:
	struct Box2DSweepTestResult {
		b2Shape *shape_A;
		b2Body *body_A;
		b2Shape *shape_B;
		b2Body *body_B;
		b2Transform place_of_impact;
		float distance;
		b2DistanceOutput output;
		bool intersects;
	};
	static b2Sweep create_b2_sweep(b2Transform p_transform, b2Vec2 p_center, b2Vec2 p_motion);
	static b2DistanceOutput call_b2_distance(b2Transform p_transformA, b2Shape *shapeA, b2Transform p_transformB, b2Shape *shapeB);
	static Box2DSweepTestResult shape_cast(b2Transform p_transformA, b2Shape *shapeA, b2Body *bodyA, b2Sweep p_sweepA, b2Transform p_transformB, b2Shape *shapeB, b2Body *bodyB, b2Sweep p_sweepB, float step_amount);
};
