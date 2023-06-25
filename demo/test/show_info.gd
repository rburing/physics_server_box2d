class_name ShowInfo
extends Node2D

var label: Label = Label.new()
var node2d: Node2D = Node2D.new()
var theme: Theme = load("res://new_theme.tres")

# Called when the node enters the scene tree for the first time.
func _ready():
	label.theme = theme
	add_child(node2d)
	node2d.add_child(label)

func show_info(info_lines: Array[String]):
	node2d.global_rotation = 0
	label.text = ""
	for info_line in info_lines:
		label.text += info_line + "\n"


