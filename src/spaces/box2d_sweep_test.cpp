#include "box2d_sweep_test.h"

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_time_of_impact.h>

b2Sweep Box2DSweepTest::create_b2_sweep(b2Transform p_transform, b2Vec2 p_center, b2Vec2 p_motion) {
	b2Sweep sweep;
	sweep.a0 = p_transform.q.GetAngle();
	sweep.a = p_transform.q.GetAngle();
	sweep.localCenter = p_center;
	sweep.c0 = sweep.localCenter + p_transform.p;
	sweep.c = sweep.localCenter + p_transform.p + p_motion;
	sweep.alpha0 = 0;
	return sweep;
}
Box2DSweepTest::Box2DSweepTestResult Box2DSweepTest::shape_cast(b2Transform p_transformA, b2Shape *shapeA, b2Body *bodyA, b2Sweep p_sweepA, b2Transform p_transformB, b2Shape *shapeB, b2Body *bodyB, b2Sweep p_sweepB, float step_amount) {
	b2TOIInput input;
	b2TOIOutput output;
	input.tMax = step_amount;
	input.sweepA = p_sweepA;
	input.sweepB = p_sweepB;
	input.proxyA.Set(shapeA, 0);
	input.proxyB.Set(shapeB, 0);
	b2TimeOfImpact(&output, &input);
	bool is_colliding = output.state != b2TOIOutput::e_overlapped || output.state != b2TOIOutput::e_touching;
	if (is_colliding) {
		p_sweepA.GetTransform(&p_transformA, step_amount);
		p_sweepB.GetTransform(&p_transformB, step_amount);
		b2DistanceOutput output = call_b2_distance(p_transformA, shapeA, p_transformB, shapeB);
		float distance = (p_transformA.p + output.pointA - p_sweepA.c0).Length();
		return Box2DSweepTestResult{ shapeA, bodyA, shapeB, bodyB, p_transformA, distance, output, true };
	}
	return Box2DSweepTestResult{ nullptr, nullptr, nullptr, nullptr, b2Transform(), 0.0f, b2DistanceOutput(), false };
}

b2DistanceOutput Box2DSweepTest::call_b2_distance(b2Transform p_transformA, b2Shape *shapeA, b2Transform p_transformB, b2Shape *shapeB) {
	b2DistanceOutput output;
	b2DistanceInput input;
	b2SimplexCache cache;
	input.proxyA.Set(shapeA, 0);
	input.proxyB.Set(shapeB, 0);
	input.transformA = p_transformA;
	input.transformA = p_transformB;
	cache.count = 0;
	b2Distance(&output, &cache, &input);
	return output;
}
