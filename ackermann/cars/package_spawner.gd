extends Node3D

@export var package_count := 1
@export var package_size := Vector3(0.5, 0.5, 0.5)
@export var shelf_size := Vector2(5.0, 5.0)
@export var top_clearance := 0.005
@export var package_spacing := 0.15
@export var package_collision_layer := 8
@export var package_collision_mask := 1

const PACKAGE_COLORS := [
	Color(0.04, 0.20, 0.85, 1.0),
	Color(0.987, 0.143, 0.0, 1.0),

]

var rng := RandomNumberGenerator.new()

func _ready() -> void:
	rng.randomize()
	_spawn_packages()

func _spawn_packages() -> void:
	var half_x := shelf_size.x * 0.5 - package_size.x * 0.5
	var half_z := shelf_size.y * 0.5 - package_size.z * 0.5
	var center_y := package_size.y * 0.5 + top_clearance
	var placed_positions: Array[Vector3] = []

	for index in range(package_count):
		var package_body := StaticBody3D.new()
		package_body.name = "Package%d" % (index + 1)
		package_body.collision_layer = package_collision_layer
		package_body.collision_mask = package_collision_mask
		package_body.position = _random_package_position(
			0.25,
			0,
			0.2,
			placed_positions
		)
		placed_positions.append(package_body.position)

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

func _random_package_position(
	half_x: float,
	half_z: float,
	center_y: float,
	placed_positions: Array[Vector3]
) -> Vector3:
	var min_distance: float = max(package_size.x, package_size.z) + package_spacing
	for _attempt in range(50):
		var candidate := Vector3(
			rng.randf_range(-half_x * 0.25, half_x * 0.25),
			center_y,
			rng.randf_range(-half_z * 0.25, half_z * 0.25)
		)
		var overlaps := false
		for placed in placed_positions:
			if Vector2(candidate.x, candidate.z).distance_to(Vector2(placed.x, placed.z)) < min_distance:
				overlaps = true
				break
		if not overlaps:
			return candidate

	return Vector3(
		rng.randf_range(-half_x, half_x),
		center_y,
		rng.randf_range(-half_z, half_z)
	)

func _package_material() -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = PACKAGE_COLORS[rng.randi_range(0, PACKAGE_COLORS.size() - 1)]
	material.roughness = 0.65
	return material
