extends Node3D

@export var package_size := Vector3(0.5, 0.5, 0.5)
@export var package_position := Vector2(0.0, 0.0)
@export var top_clearance := 0.005
@export var package_collision_layer := 8
@export var package_collision_mask := 1

const PACKAGE_COLORS := [
	Color(0.04, 0.20, 0.85, 1.0),
	Color(0.987, 0.143, 0.0, 1.0),
]

var rng := RandomNumberGenerator.new()

func _ready() -> void:
	rng.randomize()
	_spawn_package()

func _spawn_package() -> void:
	var center_y := package_size.y * 0.5 + top_clearance
	var package_body := StaticBody3D.new()
	package_body.name = "Package"
	package_body.collision_layer = package_collision_layer
	package_body.collision_mask = package_collision_mask
	package_body.position = Vector3(package_position.x, center_y, package_position.y)

	var mesh_instance := MeshInstance3D.new()
	var box_mesh := BoxMesh.new()
	box_mesh.size = package_size
	box_mesh.material = _package_material()
	mesh_instance.mesh = box_mesh
	package_body.add_child(mesh_instance)

	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = package_size
	collision.shape = shape
	package_body.add_child(collision)

	add_child(package_body)

func _package_material() -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = PACKAGE_COLORS[rng.randi_range(0, PACKAGE_COLORS.size() - 1)]
	material.roughness = 0.65
	return material
