##a better version of AStar3D
@tool class_name AStar3DModified extends AStar3D

const RADIUS_BUCKET: float = 0.25 ##agent radiuses are snapped to this so cached edge/clearance checks are reused
const PUSH_STEPS: int = 3 ##amount of passes to push a test point away from obstacles before it gives up
const MAX_RESULTS: int = 8 ##the maximum number of shapes returned from the hits for resolving the position of a point

var space: PhysicsDirectSpaceState3D ##the space state to query against
var radius: float = 0.0 ##radius currently being pathed for
var ver: int = 0 ##counter that increments whenever points move or change disable status

var _sphere: SphereShape3D = SphereShape3D.new() ##reused sphere for shape queries
var _squery: PhysicsShapeQueryParameters3D = PhysicsShapeQueryParameters3D.new() ##reused shape query to improve performance
var _rquery: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.new() ##reused ray query to improve performance

var _edge: Dictionary[float, Dictionary] ##radius bucket : {point id : {other point id : bool}} where bool is if the path between the 2 points is clear
var _resolved: Dictionary[float, Dictionary] ##radius bucket : {point id : Vector3} where the vector is the resolved position of the point
var _cver: int = -1 ##the version the cache was last built on

func _init() -> void: 
	neighbor_filter_enabled = true #allow the _filter_neighbor func to be used
	_squery.shape = _sphere #add the sphere to the query ones

##filters whether neighboors can be used or not, skipping paths between points too thin for the
##current radius- if true a point wont be processed
func _filter_neighbor(from_id: int, neighbor_id: int) -> bool:
	if space == null: return false #if there's no space tracking, the point is always valid
	var b: float = ceilf(maxf(radius, 0.0) / RADIUS_BUCKET) * RADIUS_BUCKET #snap the radius to a bucket
	if ver != _cver: #if the geometry changed, clear the buckets
		_edge = {}
		_resolved = {}
		_cver = ver
	#get the bucket's edge and resolved caches, making new ones if they didnt exist before
	if !_edge.has(b):     _edge[b] = {}
	if !_resolved.has(b): _resolved[b] = {}
	var e: Dictionary = _edge[b]
	var r: Dictionary = _resolved[b]
	#if the point pair already exists, return the cached clear value
	if e.get(from_id, {}).has(neighbor_id): return !e[from_id][neighbor_id]
	var c: bool #if the path is clear or not
	if b > 0.0: #if the radius is larger than 0, created the resolved positions
		if !r.has(from_id):     r[from_id] = _resolve_pos(get_point_position(from_id), b)
		if !r.has(neighbor_id): r[neighbor_id] = _resolve_pos(get_point_position(neighbor_id), b)
		#run a shape query with sphere of the snapped radius that travels the length of the path between the 2 points
		c = _sweep(r[from_id], r[neighbor_id], b)
	else: #if not, just use a raycast
		c = is_clear(get_point_position(from_id), get_point_position(neighbor_id), 0.0)
	#add the point pairs to the edges cache with the clear value
	if !e.has(from_id):     e[from_id] = {}
	if !e.has(neighbor_id): e[neighbor_id] = {}
	e[from_id][neighbor_id] = c
	e[neighbor_id][from_id] = c
	return !c #return the oppisote of clear so if it wasnt clear, true would be returned and the point wont be processed

##returns if a sphere of radius r can pass between 2 points without hitting anything
func _sweep(from: Vector3, to: Vector3, r: float) -> bool:
	#use the query with the radius and in a motion from the agent position to the target point 
	#position- the path is clear if there were no overlaps, or the safe fraction (element 1 
	#in the return list) is 1, meaning that the shape can travel the entire motion without hitting anything
	_sphere.radius = r
	_squery.transform = Transform3D(Basis(), from)
	_squery.motion = to - from
	var h: Array = space.cast_motion(_squery)
	return h.is_empty() or h[0] >= 1.0

##pushes a sphere of radius r at position b out of anything it overlaps and returning that
##new position of the sphere
func _resolve_pos(b: Vector3, r: float) -> Vector3:
	var res: Vector3 = b #the new position to return
	_sphere.radius = r
	for i: int in PUSH_STEPS: #repeat a number of times so the sphere is guranteed to be free
		#update the position of the sphere to the current position to be checked then run the query
		#which returns where the sphere hits other collision objects
		_squery.transform = Transform3D(Basis(), res)
		var h: PackedVector3Array = space.collide_shape(_squery, MAX_RESULTS)
		if h.is_empty(): break #if it hit nothing, the sphere is already clear so nothing else should be done
		var p: Vector3 = Vector3.ZERO #the amount to increment the result position by
		var c: int = 0 #contacts that pushed, used for averaging
		#because the hit returns something like [point on sphere, point on other shape, ...] in that
		#pattern, it iterates thru the odd entries so only the "point on other shape" values are gotten
		for k: int in range(1, h.size(), 2):
			var to: Vector3 = res - h[k] #vector pointing away from contact to the sphere's center (away from obstacle)
			var d: float = to.length() #distance from the center to the contact point
			#if the contact is closer than the radius, that indicates an overlap so a push is applied
			#that pushes the sphere away from the contact point by the difference between r and d so 
			#it'll no longer be overlapping (the 0.0001 check prevents points exactly on the sphere's
			#center from being processed)
			if d < r and d > 0.0001: 
				p += to.normalized() * (r - d)
				c += 1
		if c == 0: break #prevents dividing by 0- also there's no points to even push by anyways
		res += p / c #add the push to the result and normalize with count
	return res #return the new pushed position

##calls set_point_disabled, but updates the version too
func set_disabled(id: int, disabled: bool = true) -> void:
	if has_point(id) and is_point_disabled(id) != disabled:
		set_point_disabled(id, disabled)
		ver += 1

##calls get_id_path, but updates the space and radius; space and radius are also updated for the
##path resolving with filter neighboor
func find_path(from: Vector3, to: Vector3, _radius: float, _space: PhysicsDirectSpaceState3D) -> PackedInt64Array:
	space = _space
	radius = _radius
	var a: int = get_closest_point(from)
	var b: int = get_closest_point(to)
	return PackedInt64Array() if a == -1 or b == -1 else get_id_path(a, b) #prevent bad stuff from happening if there are no points

##returns if a sphere of radius r can pass between 2 points without hitting anything
##but also does a normal raycast if the radius is 0 or below
func is_clear(from: Vector3, to: Vector3, r: float) -> bool:
	if space == null: return false #if there's no space, it cant do anything
	if r <= 0.0: #if there's no radius just use a normal raycast- the path is clear if the ray doesnt return any hits
		_rquery.from = from
		_rquery.to = to
		return space.intersect_ray(_rquery).is_empty()
	return _sweep(from, to, r) #runs the shape query
