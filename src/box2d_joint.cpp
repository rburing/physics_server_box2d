#include "box2d_joint.h"
#include "box2d_body.h"
#include "box2d_space.h"
#include "box2d_type_conversions.h"

#include <box2d/b2_distance_joint.h>
#include <box2d/b2_prismatic_joint.h>
#include <box2d/b2_revolute_joint.h>
#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DJoint::clear() {
	configured = false;
	body_a = nullptr;
	body_b = nullptr;
	if (space) {
		space->remove_joint(this);
	}
}

void Box2DJoint::_recreate_joint() {
	if (space) {
		space->create_joint(this);
	}
}

void Box2DJoint::set_bias(real_t p_data) {
	bias = p_data;
}
void Box2DJoint::set_max_bias(real_t p_data) {
	max_bias = p_data;
}
void Box2DJoint::set_max_force(real_t p_data) {
	max_force = p_data;
}
real_t Box2DJoint::get_bias() {
	return bias;
}
real_t Box2DJoint::get_max_bias() {
	return max_bias;
}
real_t Box2DJoint::get_max_force() {
	return max_force;
}

void Box2DJoint::set_disable_collisions(bool p_disable_collisions) {
	disable_collisions = p_disable_collisions;
	joint_def->collideConnected = !disable_collisions;
	_recreate_joint();
}

bool Box2DJoint::get_disable_collisions() {
	return disable_collisions;
}

void Box2DJoint::make_pin(const Vector2 &p_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_PIN;
	anchor_a = godot_to_box2d(p_anchor);
	b2RevoluteJointDef *revolute_joint_def = memnew(b2RevoluteJointDef);
	revolute_joint_def->collideConnected = disable_collisions;
	memdelete(joint_def);
	joint_def = revolute_joint_def;
	body_a = p_body_a;
	body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::make_groove(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_GROOVE;
	groove_lower_translation = godot_to_box2d((p_a_groove2 - p_b_anchor).length());
	groove_upper_translation = godot_to_box2d((p_a_groove1 - p_b_anchor).length());
	Vector2 axis = (p_a_groove1 - p_a_groove2).normalized();
	groove_axis = b2Vec2(axis.x, axis.y);
	anchor_b = godot_to_box2d(p_b_anchor);
	b2PrismaticJointDef *prismatic_joint_def = memnew(b2PrismaticJointDef);
	prismatic_joint_def->collideConnected = disable_collisions;
	memdelete(joint_def);
	joint_def = prismatic_joint_def;
	body_a = p_body_a;
	body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::make_damped_spring(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_DAMPED_SPRING;
	anchor_a = godot_to_box2d(p_anchor_a);
	anchor_b = godot_to_box2d(p_anchor_b);
	b2DistanceJointDef *distance_joint_def = memnew(b2DistanceJointDef);
	distance_joint_def->collideConnected = disable_collisions;
	memdelete(joint_def);
	joint_def = distance_joint_def;
	body_a = p_body_a;
	body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::set_pin_softness(real_t p_softness) {
	pin_softness = p_softness; // unused
}

real_t Box2DJoint::get_pin_softness() {
	return pin_softness;
}

void Box2DJoint::set_damped_spring_rest_length(real_t p_damped_spring_rest_length) {
	damped_spring_rest_length = godot_to_box2d(p_damped_spring_rest_length);
	if (joint && (b2DistanceJoint *)joint) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		distance_joint->SetLength(damped_spring_rest_length);
	}
}
real_t Box2DJoint::get_damped_spring_rest_length() {
	return box2d_to_godot(damped_spring_rest_length);
}

void Box2DJoint::set_damped_spring_stiffness(real_t p_damped_spring_stiffness) {
	damped_spring_stiffness = godot_to_box2d(p_damped_spring_stiffness);
	if (joint && (b2DistanceJoint *)joint) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		float stiffness = 0;
		float damping = 0;
		b2LinearStiffness(stiffness, damping, damped_spring_stiffness, damped_spring_damping, body_a->get_b2Body(), body_b->get_b2Body());
		distance_joint->SetStiffness(stiffness);
	}
}
real_t Box2DJoint::get_damped_spring_stiffness() {
	return box2d_to_godot(damped_spring_stiffness);
}

void Box2DJoint::set_damped_spring_damping(real_t p_damped_spring_damping) {
	damped_spring_damping = godot_to_box2d(p_damped_spring_damping);
	if (joint && (b2DistanceJoint *)joint) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		float stiffness = 0;
		float damping = 0;
		b2LinearStiffness(stiffness, damping, damped_spring_stiffness, damped_spring_damping, body_a->get_b2Body(), body_b->get_b2Body());
		distance_joint->SetStiffness(damping);
	}
}
real_t Box2DJoint::get_damped_spring_damping() {
	return box2d_to_godot(damped_spring_damping);
}

Box2DBody *Box2DJoint::get_body_a() {
	return body_a;
}
Box2DBody *Box2DJoint::get_body_b() {
	return body_b;
}

b2JointDef *Box2DJoint::get_b2JointDef() {
	switch (type) {
		case PhysicsServer2D::JointType::JOINT_TYPE_PIN: {
			b2RevoluteJointDef *revolute_joint_def = (b2RevoluteJointDef *)joint_def;
			revolute_joint_def->enableMotor = true;
			revolute_joint_def->Initialize(body_a->get_b2Body(), body_b->get_b2Body(), anchor_a);
		} break;
		case PhysicsServer2D::JointType::JOINT_TYPE_DAMPED_SPRING: {
			b2DistanceJointDef *distance_joint_def = (b2DistanceJointDef *)joint_def;
			b2LinearStiffness(distance_joint_def->stiffness, distance_joint_def->damping, damped_spring_stiffness, damped_spring_damping, body_a->get_b2Body(), body_b->get_b2Body());
			distance_joint_def->Initialize(body_a->get_b2Body(), body_b->get_b2Body(), anchor_a, anchor_b);

			distance_joint_def->length = damped_spring_rest_length;
			distance_joint_def->minLength = 0;
		} break;
		case PhysicsServer2D::JointType::JOINT_TYPE_GROOVE: {
			b2PrismaticJointDef *prismatic_joint_def = (b2PrismaticJointDef *)joint_def;
			prismatic_joint_def->Initialize(body_a->get_b2Body(), body_b->get_b2Body(), anchor_b, groove_axis);
			//prismatic_joint_def->lowerTranslation = std::min(groove_lower_translation, groove_upper_translation);
			//prismatic_joint_def->upperTranslation = std::max(groove_lower_translation, groove_upper_translation);
			prismatic_joint_def->lowerTranslation = -groove_lower_translation;
			prismatic_joint_def->upperTranslation = groove_upper_translation;
			prismatic_joint_def->enableLimit = true;
		} break;
		default: {
			ERR_PRINT_ONCE("UNSUPPORTED");
		}
	}
	return joint_def;
}
void Box2DJoint::set_b2JointDef(b2JointDef *p_joint_def) {
	joint_def = p_joint_def;
	_recreate_joint();
}
b2Joint *Box2DJoint::get_b2Joint() {
	return joint;
}
void Box2DJoint::set_b2Joint(b2Joint *p_joint) {
	joint = p_joint;
}

void Box2DJoint::set_space(Box2DSpace *p_space) {
	space = p_space;
}

Box2DJoint::Box2DJoint() {
	joint_def = memnew(b2JointDef);
}

Box2DJoint::~Box2DJoint() {
	memdelete(joint_def);
}