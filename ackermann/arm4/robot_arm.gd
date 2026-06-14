extends Node3D

const DDS_TYPE_INT = 1
const DDS_TYPE_FLOAT = 2

@export var dds_path: NodePath = ^"/root/DDS"
@export var camera_sender_path: NodePath = ^"/root/CameraSender"
@export var theta_0_box_path: NodePath
@export var theta_1_box_path: NodePath
@export var theta_2_box_path: NodePath
@export var theta_3_box_path: NodePath
@export var current_pose_path: NodePath
@export var camera_view_path: NodePath
@export var publish_tick := true
@export var publish_camera_on_request := true

@onready var joint_base: Node3D = $Joint_base
@onready var joint_1: Node3D = $Joint_base/Joint_1
@onready var joint_3: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3
@onready var joint_5: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3/arm_2/joint_4/joint_5

var dds: Node
var camera_sender: Node
var theta_0_box: TextEdit
var theta_1_box: TextEdit
var theta_2_box: TextEdit
var theta_3_box: TextEdit
var current_pose: Label
var camera_view: TextureRect


func _ready() -> void:
	dds = get_node_or_null(dds_path)
	camera_sender = get_node_or_null(camera_sender_path)
	theta_0_box = _get_optional_node(theta_0_box_path) as TextEdit
	theta_1_box = _get_optional_node(theta_1_box_path) as TextEdit
	theta_2_box = _get_optional_node(theta_2_box_path) as TextEdit
	theta_3_box = _get_optional_node(theta_3_box_path) as TextEdit
	current_pose = _get_optional_node(current_pose_path) as Label
	camera_view = _get_optional_node(camera_view_path) as TextureRect

	if dds == null:
		push_warning("RobotArm: DDS autoload not found. Add dds.gd as /root/DDS to receive robot commands.")
		return

	for var_name in ["theta0", "theta1", "theta2", "theta3", "x", "y", "z", "a", "read_image"]:
		dds.call("subscribe", var_name)


func _process(delta: float) -> void:
	if dds == null:
		return

	if publish_tick:
		dds.call("publish", "tick", DDS_TYPE_FLOAT, delta)

	if publish_camera_on_request:
		_send_requested_camera_frame()

	var t0 = dds.call("read", "theta0")
	var t1 = dds.call("read", "theta1")
	var t2 = dds.call("read", "theta2")
	var t3 = dds.call("read", "theta3")
	var x = dds.call("read", "x")
	var y = dds.call("read", "y")
	var z = dds.call("read", "z")
	var a = dds.call("read", "a")

	if t0 != null:
		joint_base.rotation.y = float(t0)
		_set_text(theta_0_box, "%.2f" % rad_to_deg(float(t0)))
	if t1 != null:
		joint_1.rotation.y = PI / 2.0 + float(t1)
		_set_text(theta_1_box, "%.2f" % rad_to_deg(float(t1)))
	if t2 != null:
		joint_3.rotation.x = float(t2)
		_set_text(theta_2_box, "%.2f" % rad_to_deg(float(t2)))
	if t3 != null:
		if float(t3) >= 0.0:
			joint_5.rotation.z = PI / 2.0
			joint_5.rotation.y = PI / 2.0
		else:
			joint_5.rotation.z = -PI / 2.0
			joint_5.rotation.y = -PI / 2.0
		joint_5.rotation.x = -PI / 2.0 + float(t3)
		_set_text(theta_3_box, "%.2f" % rad_to_deg(float(t3)))

	if current_pose != null and x != null and y != null and z != null and a != null:
		current_pose.text = "X=%.3f   Y=%.3f   Z=%.3f   A=%.2f" % [x, y, z, rad_to_deg(float(a))]


func _send_requested_camera_frame() -> void:
	var read_image = dds.call("read", "read_image")
	if read_image == null or int(read_image) != 1:
		return

	var subscribed_vars = dds.get("subscribed_vars")
	if subscribed_vars is Dictionary:
		subscribed_vars["read_image"] = 0

	if camera_sender == null or camera_view == null or camera_view.texture == null:
		return

	var image = camera_view.texture.get_image()
	if image == null:
		return

	camera_sender.call("send_data", image.get_data())


func _get_optional_node(path: NodePath) -> Node:
	if String(path) == "":
		return null
	return get_node_or_null(path)


func _set_text(control: TextEdit, value: String) -> void:
	if control != null:
		control.text = value
