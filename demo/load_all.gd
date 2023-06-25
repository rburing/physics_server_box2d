extends Node2D

@export var scenes: Array[PackedScene]
@export var label: Label

func _ready():
	for scene in scenes:
		var instantiated := scene.instantiate()
		add_child(instantiated)
		await get_tree().create_timer(instantiated.get_meta("run_time")).timeout
		instantiated.queue_free()
	get_tree().quit()

func _process(delta):
	label.text = str(Performance.get_monitor(Performance.TIME_FPS))
