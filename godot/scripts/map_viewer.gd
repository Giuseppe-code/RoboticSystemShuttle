extends Node3D


func _ready() -> void:
	$Camera3D.look_at(Vector3(0.0, 60.0, 0.0), Vector3.UP)
