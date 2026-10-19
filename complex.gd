##node containing all spatial nodes and managing the navigation system
@tool class_name Complex extends Node

const TOLERANCE_AGENT: float = 1.5 ##spacing multiplier a agent can be from it's path segment before a new path is made
const TOLERANCE_TARGET: float = 0.5 ##spacing multiplier a target can move before a new path is made
const ADVANCE_THRESHOLD: float = 0.5 ##the minimum distance an agent has to be to a point on their path line to get the next point on the line
const SNAP_MARGIN: float = 0.9 ##multiplier of spacing to check for snapping- larger means that points farther away will get snapped, reducing density
const MAX_SHORTCUT: int = 8 ##caps how many points in a path can be skipped within line of sight

const SURFACE_ID_OFFSET: int = 1 << 32 ##surface points start here so they;re always seperate from grid point ids
const SURFACE_POS_OFFSET: float = 0.1 ##how far points are raised from their surfaces, in meters

const REFRESH_DELAY: float = 0.1 ##the time, in seconds, between refreshes

enum Mode { GRID, SURFACE } ##GRID does navigation using a 3D grid of points while SURFACE uses 2D grids of points across all shape surfaces

signal refreshed ##emmits whenever refresh is called

@export var size: int = 1: ##the width/height/depth of the grid, # of points is size ** 3
	set(v): #regenerates grid of every set
		if size == v: return
		size = v
		generate()
@export_custom(PROPERTY_HINT_NONE, "suffix:m") var spacing: float = 1.0: ##the distance inbetween points
	set(v): #regenerates grid of every set
		if spacing == v: return
		spacing = v
		generate()
@export_range(1.0, 3.0, 0.05) var connect_margin: float = 1.0 ##scalar of spacing that determines whether points get connected in SURFACE mode
@export_tool_button("Reset") var reset: Callable = _reset ##calls the reset function, clearing all associated nodes and caches

var paths: Dictionary[Node3D, Path] ##all of the path states requested by navigation agents (any Node3D that calls next_point)
var points: PackedInt64Array: ##all of the point ids from both the grid and surface astars
	get(): return _astar_grid.get_point_ids() + _astar_surface.get_point_ids()

var _astar_grid:    AStar3DModified = AStar3DModified.new() ##the navigation brain storing the grid of points
var _astar_surface: AStar3DModified = AStar3DModified.new() ##the navigation brain storing the point grids on surfaces
var _tick: int = 0 ##tracks the amount of refreshes that have occured
#NOTE: yes I understand very well that the boolean value is never used whatsoever but Dictionarys
#      run faster than Arrays, and since there's a lot of points, performance is everything!!
var _disabled: Dictionary[int, bool] ##cache of the points that were disabled between refreshes

var _buckets: Dictionary[Vector3i, Array] ##hashing of spacing-sized cells to the points inside them for finding points near obstacles or near each other
var _p_bucket: Dictionary[int, Vector3i] ##point id to the bucket it's in
var _nid: int = SURFACE_ID_OFFSET ##the next free point id, for generation

var _shapes: Dictionary[CollisionShape3D, Shape] ##a cache of the shape states of registered CollisionShape3Ds in the scene
var _shape_add: Callable = func(node: Node) -> void: ##adds a CollisionShape3D to the shapes cache paired to a generated Shape state
	if node is CollisionShape3D: 
		var shape: CollisionShape3D = node
		if shape.shape: _shapes[shape] = Shape.new(shape.shape)
var _shape_remove: Callable = func(node: Node) -> void: ##removes a CollisionShape3D from the shape cache and disconnects it's changed signal and removes its surface points
	if node is CollisionShape3D:
		var s: Shape = _shapes.get(node)
		if s: 
			s.disconnect_shape()
			_remove_shape_pts(s)
		_shapes.erase(node)

#NOTE: originally I was going to do paths.set.bind(agent, state) and paths.erase.bind(agent)
#      directly in the signals that use them, but that doesnt work because for connecting and
#      disconnecting they need to be the same Callable and also Dictionary doesnt nativley support
#      set and erase as Callables for signals to use I think
var _path_add: Callable = func(agent: Node3D, state: Path) -> void: paths.set(agent, state) ##adds a path to the path cache paired with the node that requested it
var _path_remove: Callable = func(agent: Node3D) -> void: paths.erase(agent) ##removes a path from the patch cache by agent

var _timer: Timer: ##the timer used to keep track of time inbetween refreshed
	get(): #ensures that there is always a valid instance- setting up and adding the timer to the Complex
		if !is_instance_valid(_timer):
			_timer = Timer.new()
			_timer.wait_time = REFRESH_DELAY
			_timer.timeout.connect(_refresh)
			add_child(_timer)
			_timer.start()
		return _timer

func _init() -> void:
	tree_entered.connect(func() -> void: #when the Complex is added into the scene tree,
		print("starting refresh timer ", _timer) #start the timer
		get_tree().node_added.connect(_shape_add) #add/remove shapes when any node in the tree is added/removed
		get_tree().node_removed.connect(_shape_remove)
	)
	tree_exiting.connect(func() -> void: #when the Complex is removed from the scene tree,
		get_tree().node_added.disconnect(_shape_add) #disconnect the signals added when the Complex entered the tree
		get_tree().node_removed.disconnect(_shape_remove)
		_clear()
	)

##handles the obstacles and renders the grid
func _refresh() -> void:
	_tick += 1
	_handle_shapes()
	refreshed.emit()

##clears caches, disconnects shapes, re enables disabled points, and updates astar versions
func _clear() -> void:
	for i: int in _disabled: astar_from_id(i).set_disabled(i, false)
	_disabled = {}
	for s: Shape in _shapes.values(): 
		s.disconnect_shape()
		_remove_shape_pts(s)
	_shapes = {}
	paths = {}
	_astar_grid.ver += 1
	_astar_surface.ver += 1
	_tick = 0

##clears the Complex, restarts the timer, adds shapes to be tracked, then does an initial refresh
func _reset() -> void:
	_clear()
	for node: Node in find_children("*", "", true, false): #add current children of the Complex that are shapes
		_shape_add.call(node)
	_timer.start()
	_refresh()

##returns a unique ID based on the normalized position of a point in the grid- basically a
##base-[size] number system where each place represents a axis calculated from the size of the grid.
##allows me to get points and make ids without having to run expensive astar searching functions.
func _point_id(x: int, y: int, z: int) -> int: return (x * size * size) + (y * size) + z

##returns the spatial hash thingy a position falls into. any 2 points within spacing of each other
##end up in the same cell or are direct neighboors
func _bucket(p: Vector3) -> Vector3i: return Vector3i((p / spacing).floor())

#NOTE: reparenting of collision shapes is unhandled
##tracks all of the registered CollisionShape3Ds and disables the points that are inside of their
##shapes, only checking points that were disabled or are in one of their AABBs to improve performance;
##also generates surface points on the shapes for surface navigation
func _handle_shapes() -> void:
	var disabled: Dictionary[int, bool] #the new list of disabled points
	#physics stuff for the point querying that is reused
	var space: PhysicsDirectSpaceState3D = get_viewport().find_world_3d().direct_space_state
	#update the spaces for both astars
	_astar_grid.space = space
	_astar_surface.space = space
	var query: PhysicsPointQueryParameters3D = PhysicsPointQueryParameters3D.new()
	#if not disabled on this pass, query the position; if the query returns something (if not 
	#empty there's hit data, therfore something was hit), add the point to the local disabled cache
	var disable_local: Callable = func(i: int, p: Vector3) -> void:
		if !disabled.has(i):
			query.position = p
			if !space.intersect_point(query, 1).is_empty():
				disabled[i] = true
	#if the astar for the point (determined by the point id) has the point, set the disable status
	var disable_global: Callable = func(i: int, d: bool) -> void:
		astar_from_id(i).set_disabled(i, d)
	for shape: CollisionShape3D in _shapes:
		if !shape.shape: continue
		var cache: Shape = _shapes[shape] #get the shape cache
		if cache.refresh(shape.shape, shape.global_transform): #regen points if anything changed
			_gen_surface_pts(shape, cache)
			_astar_grid.ver += 1
		#the start point and end points of the global AABB relative to the grid without spacing using the bucket system
		var a: Vector3i = _bucket(cache.global.position)
		var b: Vector3i = _bucket(cache.global.end)
		#iterates through all of the points invetween the start/end points in the AABB. b+1 is used
		#so the points on the last faces are included.
		for x: int in range(a.x, b.x + 1):
			for y: int in range(a.y, b.y + 1):
				for z: int in range(a.z, b.z + 1):
					#if inside the bounds of the grid, check to disable it
					if x >= 0 and y >= 0 and z >= 0 and x < size and y < size and z < size:
						disable_local.call(_point_id(x, y, z), Vector3(x, y, z) * spacing)
					#iterate through every surface point in the current cell, skipping already disabled 
					#points and for each check to disable it
					for i: int in _buckets.get(Vector3i(x, y, z), []):
						disable_local.call(i, _astar_surface.get_point_position(i))
	for i: int in disabled: #all the points that were disabled in this pass
		if !_disabled.has(i): disable_global.call(i, true) #if it's newly disabled, make it disabled
	for i: int in _disabled: #all of the points that were disabled in the last pass
		if !disabled.has(i): disable_global.call(i, false) #if it's not disabled anymore in this pass, re enable it
	_disabled = disabled #make the local cache the global cache

#NOTE: this does not produce perfectly uniform 3D grids across and between shapes, so
# expect misalignment and curved faces with higher densities
##builds the surface points for a single shape, and connects new points to existing points to ones of
##this shape or others within range
func _gen_surface_pts(shape: CollisionShape3D, cache: Shape) -> void:
	_remove_shape_pts(cache) #clears the points of the shape
	var mesh: ArrayMesh = shape.shape.get_debug_mesh() #get the mesh of the shape for the surfaces
	var snap: Dictionary[Vector3i, int] #cache for point deduplication
	var new: PackedInt64Array #ids of points created
	var add: Callable = func(p: Vector3) -> void:
		#snapping so shared edge points merge (if the rounded pos already has a point, then it doesnt
		#need to be recreated
		var k: Vector3i = Vector3i((p / (spacing * SNAP_MARGIN)).round())
		if snap.has(k): return
		#get the id for the point currentley then increment the counter so it's ready for the next point
		var id: int = _nid
		_nid += 1
		_astar_surface.add_point(id, p) #add the point to the astar
		snap[k] = id #add to the snap cache for future deduping
		new.append(id) #add the id to the list of ids for points just created
		#create a bucket for the point or merge into an existing one in the spatial hash thingy
		var b: Vector3i = _bucket(p)
		_p_bucket[id] = b
		if !_buckets.has(b): _buckets[b] = []
		_buckets[b].append(id)
	for s: int in mesh.get_surface_count():
		if mesh.surface_get_primitive_type(s) != Mesh.PRIMITIVE_TRIANGLES: continue #this algorithm should only work on triangles, so if it's not a triangle skip it
		#get the arrays of vertexes and indexes of the surface, part of the surface arrays
		var arr: Array = mesh.surface_get_arrays(s)
		var v: PackedVector3Array = arr[Mesh.ARRAY_VERTEX]
		var i: PackedInt32Array   = arr[Mesh.ARRAY_INDEX]
		var tri: int = int((i.size() if !i.is_empty() else v.size()) / 3.0) #get the # of triangles on the surface- prefer to use indexes
		for t: int in tri: #for each triangle, get the 3 vertexes in global space
			var a: Vector3 = shape.global_transform * v[i[t * 3 + 0] if !i.is_empty() else t * 3 + 0]
			var b: Vector3 = shape.global_transform * v[i[t * 3 + 1] if !i.is_empty() else t * 3 + 1]
			var c: Vector3 = shape.global_transform * v[i[t * 3 + 2] if !i.is_empty() else t * 3 + 2]
			var nor: Vector3 = (b - a).cross(c - a).normalized() #normal of the surface, for the offset
			#using a barycentric grid formula thingy, add points on the area inside the triangle
			#to best explain this, assume right triangle ABC. nab is how many spacing-sized steps
			#fit on the edge AB and nac for AC
			var nab: int = maxi(1, roundi(a.distance_to(b) / spacing))
			var nac: int = maxi(1, roundi(a.distance_to(c) / spacing))
			#x counts along steps on AB and y for AC, creating a parraleogram shaped grid
			for x: int in range(nab + 1):
				for y: int in range(nac + 1):
					#each point in the paralleogram has coordinates (x / nab, y / nac). a point is
					#inside the triangle if these coordinates add up to 1 or below (because b is (1, 0) 
					#and c is (0, 1)- we just take out the fractions to make it run better
					if x * nac + y * nab <= nab * nac:
						add.call( #the point is then added in global space
							#b - a is edge AB, so if we scale it by our coordinate system from above 
							#(x / nab), this converts that x coordinate into global space. the same thing 
							#is done with y / nac along edge AC. +a is done because all the points are
							#measured from a, the corner point that AB and AC share
							(a + (b - a) * (float(x) / nab) + (c - a) * (float(y) / nac))
							#this lifts the point off of the triangle. the normal is perpendicular
							#to the triangle plane, and since I made it backwards, - is up, so
							#multiplying the normal by the offset amount and subtracting that from
							#the point on the surface gives you a raised point
							- (nor * SURFACE_POS_OFFSET))
	cache.pids = new #update the ids of the points in the shape cache
	for i: int in new:
		var p: Vector3 = _astar_surface.get_point_position(i) #get the current point via id
		var b: Vector3i = _bucket(p) #base bucket the current point is in
		var r: int = maxi(1, ceili(connect_margin)) #how many cells from the current to search via margin
		var rmax: float = spacing * connect_margin #the max range the 2 points can be from each other to connect
		#iterates in the radius, getting the other buckets x/y/z from the base bucket
		for x: int in range(-r, r + 1):
			for y: int in range(-r, r + 1):
				for z: int in range(-r, r + 1):
					for o: int in _buckets.get(b + Vector3i(x, y, z), []): #iterates thru the bucket's points
						if o == i or _astar_surface.are_points_connected(i, o): continue #skip same points and already connected points
						if p.distance_to(_astar_surface.get_point_position(o)) <= rmax: _astar_surface.connect_points(i, o) #if the 2 points are within range, connect them
	_astar_surface.ver += 1 #update the surface version

##removes a shape's points (any reference to them) and clears the points cache of the shape
func _remove_shape_pts(cache: Shape) -> void:
	for i: int in cache.pids:
		_astar_surface.remove_point(i)
		#removes the point out of the spatial hash thingy
		if _p_bucket.has(i):
			var b: Vector3i = _p_bucket[i]
			_buckets[b].erase(i)
			if _buckets[b].is_empty(): _buckets.erase(b)
			_p_bucket.erase(i)
	cache.pids = PackedInt64Array()
	_astar_surface.ver += 1

##loads a flattened 3D array of points onto the AStar3D grid with side lengths of size and spaced
##spacing meters apart for navigation
func generate() -> void:
	#clears everything and updates versions
	_astar_surface.clear()
	_buckets = {}
	_p_bucket = {}
	_disabled = {}
	_astar_grid.ver += 1
	_astar_surface.ver += 1
	_nid = SURFACE_ID_OFFSET
	for s: Shape in _shapes.values(): 
		s.pids = PackedInt64Array()
		s.dirty = true
	#clears the grid and reserves space for the grid (saves memory since the size is defined)
	_astar_grid.clear()
	_astar_grid.reserve_space(size * size * size)
	for x: int in range(size):
		for y: int in range(size):
			for z: int in range(size):
				var i: int = _point_id(x, y , z) #get the id for the point using the current x/y/z values
				_astar_grid.add_point(i, Vector3(x, y, z) * spacing) #add the point to the grid
				#in order for the algorithm to go between points, they have to be connected. the current
				#point is connected to the previous point on each of it's 3 axises if it's not the first
				#for that axis (though there will look like there's 6 connections on each inner point
				#in the end, the next point on the axis hasnt been created yet so we cannot connect it)
				#resulting in all points connected to their neighbors in all 6 directions
				if x > 0: _astar_grid.connect_points(i, _point_id(x - 1, y, z))
				if y > 0: _astar_grid.connect_points(i, _point_id(x, y - 1, z))
				if z > 0: _astar_grid.connect_points(i, _point_id(x, y, z - 1))

##called by agents, this gets the next point the agent should go to to reach the target position
func next_point(agent: Node3D, target: Vector3, mode: Mode, radius: float = 0.0) -> Vector3:
	#get the space for physics querying and also update the space of the astar for the path
	var space: PhysicsDirectSpaceState3D = get_viewport().find_world_3d().direct_space_state
	var astar: AStar3DModified = astar_from_mode(mode)
	var p: Path = paths.get(agent) #gets the path cached for the agent if it exists
	if p == null: #if it doesnt, make a new one
		p = Path.new()
		paths[agent] = p
		#connect the signals to add the path to the cache when the agent enters the tree and
		#remove it when the agent exists the tree to prevent stale references
		var add: Callable = _path_add.bind(agent, p)
		var remove: Callable = _path_remove.bind(agent)
		if !agent.tree_entered.is_connected(add): agent.tree_entered.connect(add)
		if !agent.tree_exiting.is_connected(remove): agent.tree_exiting.connect(remove)
	if p.tick != _tick: #recalculations only happen every refresh because theyre expensive
		p.tick = _tick
		#a recalculation should be done if the path is empty or the version, mode, or target moved too much
		var r: bool = p.ids.is_empty() or p.ver != astar.ver or p.mode != mode or p.target.distance_to(target) > spacing * TOLERANCE_TARGET
		if !r: #if no redo was requested, update r to a stray check
			#get the current point the agent is going to, ensuring it doesnt go out of bounds
			var c: Vector3 = astar.get_point_position(p.ids[mini(p.idx, p.ids.size() - 1)])
			#if the agent no longer has a line of sight to the agent isnt near the lane/path it's supposed
			#to be going on- determined by getting the closest point on the line segment to the agent, and
			#if past the stray distance threshold, then, and for both conditions, stray is set to true
			#to force a recalculation of the path
			r = (p.idx > 0 and agent.global_position.distance_to(
				Geometry3D.get_closest_point_to_segment(
					agent.global_position, 
					astar.get_point_position(p.ids[p.idx - 1]), 
					c)
			) > spacing * TOLERANCE_AGENT) or !astar.is_clear(agent.global_position, c, radius)
		if r: #if a redo is still imminent,
			#update the target position/mode/radius, calculate a new path from the agent to the target,
			#reset the indexing (the agent is now at the beginning of the path) and update the version
			p.mode = mode
			p.target = target
			p.radius = radius
			p.ids = astar.find_path(agent.global_position, target, radius, space)
			p.idx = 0
			p.ver = astar.ver
		if p.ids.is_empty(): return agent.global_position #if there's still no path, just return the position of the agent so it stays still
		#theta* inspired thingy
		var i: int = 0 #number of shortcuts made
		#while there are still points in the path and the shortcut max hasnt been exceeded, run a
		#query from the agent to the point of intrest
		while p.idx < p.ids.size() - 1 and i < MAX_SHORTCUT:
			#get the to point and it's position
			var to: int = p.ids[p.idx + 1]
			var pos: Vector3 = astar.get_point_position(to)
			#if there is a open line of sight, set the next point to go to as the
			#current point of intrest, but if it doesnt end the loop
			if astar.is_clear(agent.global_position, pos, radius):
				p.idx += 1
				i += 1
			else: break
	if p.ids.is_empty(): return agent.global_position #another no-path check
	if p.idx >= p.ids.size() - 1: return target #if the next target is past or at the last point, the target, return it
	#if not an edge case, get the next point position for the agent to go to- if the agent is close
	#enough to the point to advance, update the next point position to the next one on the path
	var next: Vector3 = astar.get_point_position(p.ids[p.idx])
	if agent.global_position.distance_to(next) <= ADVANCE_THRESHOLD:
		p.idx += 1
		next = astar.get_point_position(p.ids[p.idx])
	return next #return this next position to go to

##returns the astar to use based on the point's ID
func astar_from_id(id: int) -> AStar3DModified: return _astar_grid if id < SURFACE_ID_OFFSET else _astar_surface

##returns the astar to use based on the mode passed in
func astar_from_mode(mode: Mode) -> AStar3DModified: return _astar_grid if mode == Mode.GRID else _astar_surface

##class storing a path state for each agent
class Path extends RefCounted:
	var mode: Mode ##the type of astar the path should be gotten from
	var target: Vector3 ##the last target position requested
	var radius: float ##the agent's size
	var ids: PackedInt64Array ##the points, in order, to get from the agent position to the target position
	var idx: int = 0 ##the current point the agent must go to next
	var ver: int = -1 ##the last recorded version of the Complex the path is running on
	var tick: int = -1 ##the last recorded tick of the Complex the path is running on

##class storing a shape state for a CollisionShape3D
class Shape extends RefCounted:
	var shape: Shape3D ##the raw shape resource of the CollisionShape3D
	var local: AABB ##the AABB of the shape local to the CollisionShape3D
	var trans: Transform3D ##the last saved transform of the CollisionShape3D
	var global: AABB ##the local AABB converted into global space via transformation
	var dirty: bool ##if shape changed (resource or values), requiring a refresh on the next get
	var pids: PackedInt64Array ##surface points generated for the shape
	
	func _init(_shape: Shape3D) -> void: _update(_shape) #updates on initization
	
	##swaps the signal of the old shape with the new one if reconnect is true (2 functionalities 
	##merged into 1 call to reduce function calls), gets the AABB of it, and requests a refresh to be done
	func _update(_shape: Shape3D, reconnect: bool = true) -> void: 
		if reconnect: 
			disconnect_shape()
			shape = _shape
			shape.changed.connect(_update.bind(shape, false))
		local = shape.get_debug_mesh().get_aabb()
		dirty = true
	
	##updates the shape/transformation caches if the new values passed in are different, calculates
	##the global AABB, and returns if something changed
	func refresh(_shape: Shape3D, _trans: Transform3D) -> bool:
		if _shape != shape: _update(_shape)
		if dirty or _trans != trans:
			trans = _trans
			global = trans * local
			dirty = false #changed have been accounted for so dirty can be cleared
			return true
		return false
	
	##disconnects the updated function from the changed signal of the shape
	func disconnect_shape() -> void:
		if is_instance_valid(shape) and shape.changed.is_connected(_update.bind(shape, false)):
			shape.changed.disconnect(_update.bind(shape, false))
