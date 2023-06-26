extends ShowInfo

func _process(delta):
	show_info(["sleeping: %s" % str(self.sleeping)])
