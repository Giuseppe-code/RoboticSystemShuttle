extends Camera3D

@export var source_camera_path: NodePath
var source_camera: Camera3D

func _ready() -> void:
	source_camera = get_node_or_null(source_camera_path) as Camera3D

func _process(_delta: float) -> void:
	if source_camera == null:
		return

	global_transform = source_camera.global_transform
	fov = source_camera.fov
	near = source_camera.near
	far = source_camera.far
