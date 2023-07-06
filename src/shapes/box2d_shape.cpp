#include "box2d_shape.h"
#include "bodies/box2d_collision_object.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShape::set_body(Box2DCollisionObject *p_body) {
	body = p_body;
}
Box2DCollisionObject *Box2DShape::get_body() const {
	return body;
}
void Box2DShape::add_b2_shape(b2Shape *p_shape) {
	shapes.append(p_shape);
}
Vector<b2Shape *> Box2DShape::get_b2_shapes() const {
	return shapes;
}
