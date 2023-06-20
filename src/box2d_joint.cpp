#include "box2d_joint.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DJoint::set_data(const Variant &p_data) {
}
Variant Box2DJoint::get_data() const {
	return Variant();
}
void Box2DJoint::clear() {
}

void Box2DJoint::set_disable_collisions(bool p_disable_collisions) {
	disable_collisions = p_disable_collisions;
}

bool Box2DJoint::get_disable_collisions() {
	return disable_collisions;
}

void Box2DJoint::make_pin(const Vector2 &p_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_PIN;
	configured = true;
}

void Box2DJoint::make_groove(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_GROOVE;
	configured = true;
}

void Box2DJoint::make_damped_spring(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_DAMPED_SPRING;
	configured = true;
}

void Box2DJoint::set_pin_softness(double p_softness) {
	pin_softness = p_softness;
}

double Box2DJoint::get_pin_softness() {
	return pin_softness;
}

void Box2DJoint::set_damped_spring_rest_length(double p_damped_spring_rest_length) {
	damped_spring_rest_length = p_damped_spring_rest_length;
}
double Box2DJoint::get_damped_spring_rest_length() {
	return damped_spring_rest_length;
}

void Box2DJoint::set_damped_spring_stiffness(double p_damped_spring_stiffness) {
	damped_spring_stiffness = p_damped_spring_stiffness;
}
double Box2DJoint::get_damped_spring_stiffness() {
	return damped_spring_stiffness;
}

void Box2DJoint::set_damped_spring_damping(double p_damped_spring_damping) {
	damped_spring_damping = p_damped_spring_damping;
}
double Box2DJoint::get_damped_spring_damping() {
	return damped_spring_damping;
}

Box2DBody *Box2DJoint::get_body_a() {
	return body_a;
}
Box2DBody *Box2DJoint::get_body_b() {
	return body_b;
}