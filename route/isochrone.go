package route

import (
	"fmt"
	"math"
	"sort"
	"sync"
	"time"

	log "github.com/sirupsen/logrus"

	"github.com/a-bouts/nav-server/land"
	"github.com/a-bouts/nav-server/latlon"
	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
	"github.com/a-bouts/nav-server/xmpp"
)

type Context struct {
	expes      map[string]bool
	race       Race
	polar      polar.Polar
	boat       polar.Boat
	land       *land.Land
	winds      *wind.Winds
	winchMalus float64
	latlon.LatLonSpherical
	maxDistFactor    float64
	delta            float64
	positionProvider positionProvider
}

type IsochronePosition struct {
	Latlon latlon.LatLon `json:"latlon"`
}

func (p *Position) forIsochrone() IsochronePosition {
	return IsochronePosition{
		Latlon: p.Latlon,
	}
}

type Isochrone struct {
	Color string                `json:"color"`
	Paths [][]IsochronePosition `json:"paths"`
}

type Nav struct {
	Name       string      `json:"name"`
	Isochrones []Isochrone `json:"isochrones"`
}

type Navs struct {
	Sumup    Sumup              `json:"sumup"`
	Navs     []Nav              `json:"navs"`
	WindLine []WindLinePosition `json:"windline"`
}

type Sumup struct {
	Start         time.Time        `json:"start"`
	Duration      float64          `json:"duration"`
	SailsDuration map[byte]float64 `json:"sailsDuration"`
	FoilDuration  float64          `json:"foilDuration"`
	Success       bool             `json:"success"`
}

type Position struct {
	Latlon latlon.LatLon
	//az               int
	fromDist         float64
	bearing          int
	twa              float64
	wind             float64
	windSpeed        float64
	boatSpeed        float64
	foil             uint8
	distTo           float64
	previousWindLine *Position
	duration         float64
	navDuration      float64
	doorReached      uint8
	isLand           bool
	sail             byte
	change           bool
	reached          bool
	isInIceLimits    bool
}

type WindLinePosition struct {
	Lat       float64 `json:"lat"`
	Lon       float64 `json:"lon"`
	Twa       float64 `json:"twa"`
	Bearing   int     `json:"bearing"`
	Wind      float64 `json:"wind"`
	WindSpeed float64 `json:"windSpeed"`
	BoatSpeed float64 `json:"boatSpeed"`
	Sail      byte    `json:"sail"`
	Foil      uint8   `json:"foil"`
	Ice       bool    `json:"ice"`
	Duration  float64 `json:"duration"`
	Change    bool    `json:"change"`
}

type IsLand struct {
	IsLand bool `json:"island"`
}

type positionProvider interface {
	get() *Position
	put(*Position)
}

type positionProviderPool struct {
	pool *sync.Pool
}

func (p positionProviderPool) get() *Position {
	pos := p.pool.Get().(*Position)
	pos.clear()
	return pos
}

func (p positionProviderPool) put(pos *Position) {
	p.pool.Put(pos)
}

type positionProviderNew struct {
}

func (p positionProviderNew) get() *Position {
	return new(Position)
}

func (p positionProviderNew) put(pos *Position) {
}

func (pos *Position) clear() {
	pos.fromDist = 0
	pos.bearing = 0
	pos.twa = 0
	pos.wind = 0
	pos.windSpeed = 0
	pos.boatSpeed = 0
	pos.sail = 0
	pos.foil = 0
	pos.distTo = 0
	pos.previousWindLine = nil
	pos.duration = 0
	pos.navDuration = 0
	pos.isLand = false
	pos.change = false
	pos.reached = false
	pos.doorReached = 0
	pos.isInIceLimits = false
}

func (context *Context) isExpes(expe string) bool {

	if context.expes != nil {
		if _, found := context.expes[expe]; found && context.expes[expe] {
			return true
		}
	}

	return false
}

func isToAvoid(buoy Buoy, p latlon.LatLon) bool {
	for _, t := range buoy.toAvoid() {
		as_x := p.Lat - t[0][0]
		as_y := p.Lon - t[0][1]

		s_ab := (t[1][0]-t[0][0])*as_y-(t[1][1]-t[0][1])*as_x > 0

		if (t[2][0]-t[0][0])*as_y-(t[2][1]-t[0][1])*as_x > 0 == s_ab {
			continue
		}

		if (t[2][0]-t[1][0])*(p.Lon-t[1][1])-(t[2][1]-t[1][1])*(p.Lat-t[1][0]) > 0 != s_ab {
			continue
		}

		return true
	}

	return false
}

var cartesian latlon.LatLonHaversine = latlon.LatLonHaversine{}

func jump(context *Context, start *Position, buoy Buoy, src *Position, b float64, wb float64, ws float64, d float64, factor float64, min *float64) (int, *Position) {
	change := false

	twa := float64(b) - wb
	if twa < -180 {
		twa += 360
	}
	if twa > 180 {
		twa = twa - 360
	}

	be := int(math.Round(b))
	diff := int(math.Abs(float64(be - src.bearing)))
	if diff > 180 {
		diff = 360 - diff
	}
	if false { //context.experiment {
		if math.Abs(twa) < 30 || math.Abs(twa) > 170 {
			return 0, nil
		}
	}

	bearing := int(math.Round(b))

	isInIceLimits := src.isInIceLimits
	boatSpeed, sail, isFoil := context.polar.GetBoatSpeed(twa, ws, context.boat, isInIceLimits)
	//if context.experiment {
	//	boatSpeed, sail = context.polar.GetOptimBoatSpeed(twa, ws*3.6, context.boat, src.sail, context.winchMalus)
	//}
	//if boatSpeed <= 0.0 {
	//    return 0, nil
	//}
	dist := boatSpeed * 1.852 * d * 1000.0
	if twa*src.twa < 0 || sail != src.sail {
		// changement de voile = vitesse / 2 pendant 5 minutes : on enlève 2 minutes et demi
		dist = boatSpeed * 1.852 * (d*60.0 - context.winchMalus/2) / 60 * 1000.0
		change = true

	}

	to := context.Destination(src.Latlon, float64(b), dist)
	if context.land != nil && context.land.IsLand(to.Lat, to.Lon) {
		return 0, nil
	}

	fullDist, az := 0.0, 0.0
	if context.isExpes("sqrt-dist-from") {
		fullDist, az = cartesian.DistanceAndBearingTo(start.Latlon, to)
	} else {
		fullDist, az = context.DistanceAndBearingTo(start.Latlon, to)
	}

	res := context.positionProvider.get()
	res.Latlon = to
	res.fromDist = fullDist
	res.bearing = bearing
	res.twa = twa
	res.wind = wb
	res.windSpeed = ws * 1.943844
	res.boatSpeed = boatSpeed
	res.sail = sail
	res.foil = isFoil
	res.isInIceLimits = context.race.IceLimits.isInIceLimits(&to)
	res.duration = d + src.duration
	res.navDuration = d
	res.previousWindLine = src
	res.change = change

	if buoy != nil {
		if context.isExpes("sqrt-dist-from") {
			res.distTo = cartesian.DistanceTo(to, buoy.destination())
		} else {
			res.distTo = context.DistanceTo(to, buoy.destination())
		}
		if res.distTo < *min {
			*min = res.distTo
		}
	}
	return int(math.Round(az * factor)), res
}

func doorReached(context *Context, start *Position, src *Position, buoy Buoy, wb float64, ws float64, duration float64, factor float64) (bool, float64, *Position, int) {
	distToWaypoint, az12 := context.DistanceAndBearingTo(src.Latlon, buoy.destination())

	if buoy.radius() > 0 {
		distToWaypoint -= float64(buoy.radius() * 1852)
	}

	twa := az12 - wb
	if twa < -180 {
		twa += 360
	}
	if twa > 180 {
		twa = 360 - twa
	}

	isInIceLimits := src.isInIceLimits
	boatSpeed, sail, isFoil := context.polar.GetBoatSpeed(twa, ws, context.boat, isInIceLimits)
	durationToWaypoint := (distToWaypoint / 1000.0) / (boatSpeed * 1.852)

	change := false
	if twa*src.twa < 0 || sail != src.sail {
		change = true
		//dist := (boatSpeed / 2.0) * 1.852 * 5.0 / 60.0 * 1000.0
		dist := (boatSpeed / 2.0) * 1.852 * context.winchMalus / 60.0 * 1000.0
		if change {
			if dist > distToWaypoint {
				durationToWaypoint = (distToWaypoint / 1000.0) / ((boatSpeed / 2.0) * 1.852)
			} else {
				durationToWaypoint = context.winchMalus/60.0 + ((distToWaypoint-dist)/1000.0)/(boatSpeed*1.852)
			}
		}
	}

	if durationToWaypoint <= 1.5*duration {
		latlon := buoy.destination()

		fullDist, az := 0.0, 0.0
		if context.isExpes("sqrt-dist-from") {
			fullDist, az = cartesian.DistanceAndBearingTo(start.Latlon, latlon)
		} else {
			fullDist, az = context.DistanceAndBearingTo(start.Latlon, latlon)
		}
		res := context.positionProvider.get()
		res.Latlon = latlon
		res.fromDist = fullDist
		res.bearing = int(math.Round(az12))
		res.twa = twa
		res.wind = wb
		res.windSpeed = ws * 1.943844
		res.boatSpeed = boatSpeed
		res.sail = sail
		res.foil = isFoil
		res.isInIceLimits = context.race.IceLimits.isInIceLimits(&latlon)
		res.distTo = 0
		res.duration = durationToWaypoint + src.duration
		res.navDuration = durationToWaypoint
		res.previousWindLine = src
		res.change = change

		return true, durationToWaypoint, res, int(math.Round(az * factor))
	}
	return false, 0, nil, 0
}

func way(context *Context, start *Position, src *Position, wb float64, ws float64, duration float64, buoy Buoy, isochrone map[int]*Position, factor float64, min *float64) (map[int](*Position), bool, float64) {
	result := make(map[int](*Position))

	reached, dur, point, az := doorReached(context, start, src, buoy, wb, ws, duration, factor)
	if reached {
		result[az] = point
		return result, true, dur
	}

	bMin := 0
	bMax := 360
	if context.isExpes("optim") {
		if start.fromDist > 0.0 {
			bMin = start.bearing - 120
			if bMin < 0 {
				bMin = 360 - bMin
			}
			bMax = start.bearing + 120
			if bMax >= 360 {
				bMax = bMax - 360
			}
		}
	}

	if bMax > bMin {
		for b := float64(bMin); b < float64(bMax); b += 1.0 {
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				} else {
					context.positionProvider.put(to)
				}
			}
		}
	} else {
		for b := float64(bMin); b < 360.0; b += 1.0 {
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				} else {
					context.positionProvider.put(to)
				}
			}
		}
		for b := 0.0; b < float64(bMax); b += 1.0 {
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				} else {
					context.positionProvider.put(to)
				}
			}
		}
	}

	if buoy.buoyType() == "DOOR" {
		t := context.BearingTo(buoy.(*Door).Left, buoy.(*Door).Right)
		a := context.BearingTo(src.Latlon, buoy.(*Door).Left)
		alpha := 180 + a - t
		// if a > 180 {
		// 	alpha = alpha - 360
		// }

		b := context.BearingTo(src.Latlon, buoy.(*Door).Right)
		beta := b - t
		// if b > 180 {
		// 	beta = beta - 360
		// }

		reached := false
		reachedResult := make(map[int](*Position))
		for az, res := range result {
			//check if bearing allow to cross the door
			if b < t && (a < b && float64(res.bearing) > a && float64(res.bearing) < b || a > b && (float64(res.bearing) > a || float64(res.bearing) < b)) {
				a2 := context.BearingTo(res.Latlon, buoy.(*Door).Left)
				alpha2 := 180 + a2 - t
				if a2 > 180 {
					alpha2 = alpha2 - 360
				}
				b2 := context.BearingTo(res.Latlon, buoy.(*Door).Right)
				beta2 := b2 - t
				// if b2 > 180 {
				// 	beta2 = beta2 - 360
				// }

				if alpha*alpha2 < 0 && beta*beta2 < 0 {
					//fmt.Println("t", t, "a", a, "alpha", alpha, "a2", a2, "alpha2", alpha2, "b", b, "beta", beta, "b2", b2, "beta2", beta2)
					reachedResult[az] = res
					res.reached = true
					res.doorReached = buoy.id()
				}
			}
		}
		if reached {
			return reachedResult, true, duration
		}
	}

	return result, false, duration
}

func navigate(context *Context, now time.Time, factor float64, max map[int]float64, min *float64, start *Position, previous_isochrone map[int]*Position, isochrone map[int]*Position, buoy Buoy, duration float64) (map[int]*Position, bool, float64) {
	reached := false
	var lock = sync.RWMutex{}

	minWayDuration := duration
	var wg sync.WaitGroup
	cpt := 0

	w, w1, x := context.winds.FindWinds(now)

	for _, src := range previous_isochrone {
		cpt++
		wg.Add(1)

		go func(src *Position) {

			wb, ws := wind.Interpolate(w, w1, src.Latlon.Lat, src.Latlon.Lon, x)

			way, reachedByWay, wayDuration := way(context, start, src, wb, ws, duration, buoy, isochrone, factor, min)
			if reachedByWay {
				if buoy.buoyType() == "WAYPOINT" {
					for b, dst := range way {
						lock.Lock()
						_, exists := isochrone[b]
						if !exists || len(isochrone) > 1 {
							isochrone = make(map[int]*Position)
						}
						if !exists || isochrone[b] == nil || isochrone[b].duration > wayDuration {
							isochrone[b] = dst
							minWayDuration = wayDuration
						} else {
							context.positionProvider.put(dst)
						}
						lock.Unlock()
					}
				}
				reached = true
			} else if !reached {
				for az, dst := range way {
					lock.RLock()
					prev, exists := isochrone[az]
					lock.RUnlock()
					if !exists || prev.fromDist < dst.fromDist {
						lock.Lock()
						isochrone[az] = dst
						lock.Unlock()
					} else {
						context.positionProvider.put(dst)
					}
				}
			}
			wg.Done()
		}(src)
		if cpt%15 == 0 {
			wg.Wait()
		}
	}
	wg.Wait()
	if !reached {
		toRemove := make([]int, 0, len(isochrone))
		nbLtMax := 0
		nbToAvoid := 0
		nbLand := 0
		nbFar := 0
		nbFarFromMin := 0
		for az, dst := range isochrone {
			parentReached := false
			p := dst
			for i := 0; i < 10; i++ {
				p = p.previousWindLine
				if p == nil {
					break
				}
				if p != nil && p.reached && p.doorReached == buoy.id() {
					parentReached = true
					break
				}
			}

			if dst.reached && dst.doorReached == buoy.id() || parentReached {
				buoy.reach(context, dst.previousWindLine)
			}
			d, e := max[az]
			if e && d*1.001 > dst.fromDist {
				nbLtMax++
				toRemove = append(toRemove, az)
			} else {
				maxDistFactor := context.maxDistFactor
				if context.isExpes("max-dist") {
					maxDistFactor = 1.2
					refDist := math.Min(dst.fromDist, dst.distTo)
					if refDist/1000.0 < 100.0 {
						//maxDistFactor = 1.8
						maxDistFactor = 1.5 + 0.3*(100.0-refDist/1000.0)
					} else if refDist/1000.0 < 1000.0 {
						//maxDistFactor = 1.5
						maxDistFactor = 1.2 + 0.3*(1000.0-refDist/1000.0)
					}
				}
				if isToAvoid(buoy, dst.Latlon) {
					nbToAvoid++
					toRemove = append(toRemove, az)
				} else if context.land.IsLand(dst.Latlon.Lat, dst.Latlon.Lon) {
					nbLand++
					toRemove = append(toRemove, az)
				} else if dst.fromDist+dst.distTo > maxDistFactor*start.distTo {
					nbFar++
					toRemove = append(toRemove, az)
				} else if len(isochrone)-len(toRemove) > 25 && dst.distTo > 2**min {
					nbFarFromMin++
					toRemove = append(toRemove, az)
				} else {
					max[az] = dst.fromDist
				}
			}
		}
		if nbLtMax+nbToAvoid+nbLand+nbFar+nbFarFromMin > 0 {
			log.Debugf("Remove %d < Max, %d to avoid, %d land, %d far from way, %d far from min", nbLtMax, nbToAvoid, nbLand, nbFar, nbFarFromMin)
		}
		for _, az := range toRemove {
			context.positionProvider.put(isochrone[az])
			delete(isochrone, az)
		}
	}
	return isochrone, reached, minWayDuration
}

func newPos(departure latlon.LatLon) *Position {
	return &Position{
		Latlon:   departure,
		fromDist: 0.0}
}

func newPosition(context Context, start latlon.LatLon, buoy Buoy) float64 {
	dist, _ := context.DistanceAndBearingTo(start, buoy.destination())
	log.Debugf("Go to waypoint %s (*%f - %.1f km)\n", buoy.name(), buoy.getFactor(), dist/1000.0)

	return dist
}

func findWinds(winds map[string][]*wind.Wind, m time.Time) ([]*wind.Wind, []*wind.Wind, float64) {

	stamp := m.Format("2006010215")

	keys := make([]string, 0, len(winds))
	for k := range winds {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	if keys[0] > stamp {
		return winds[keys[0]], nil, 0
	}
	for i := range keys {
		if keys[i] > stamp {
			h := m.Sub(winds[keys[i-1]][0].Date).Minutes()
			delta := winds[keys[i]][0].Date.Sub(winds[keys[i-1]][0].Date).Minutes()
			return winds[keys[i-1]], winds[keys[i]], h / delta
		}
	}
	return winds[keys[len(keys)-1]], nil, 0
}

func Run(expes map[string]bool, l *land.Land, winds *wind.Winds, xm *xmpp.Xmpp, start latlon.LatLon, bearing int, currentSail byte, race Race, delta float64, deltas map[int]float64, maxDuration float64, startTime time.Time, sail int, foil bool, hull bool, winchMalus float64, stop bool, positionPool *sync.Pool) Navs {

	context := Context{
		expes:         expes,
		race:          race,
		boat:          polar.Boat{Foil: foil, Hull: hull, Sails: sail},
		land:          l,
		winds:         winds,
		winchMalus:    winchMalus,
		maxDistFactor: 1.5,
		delta:         delta,
		positionProvider: positionProviderPool{
			pool: positionPool,
		},
	}

	if delta <= 1 {
		context.expes["progressive-intervales"] = true
		context.delta = 3
	}

	var z polar.Polar
	z = polar.Init(polar.Options{Race: race.Polars, Sail: sail})

	if context.isExpes("new-polars") {
		log.Debug("Load new polars")
		z = polar.Load(polar.Options{Race: race.Boat, Sail: sail})
	}

	context.polar = z

	buoys := race.GetBuyos(context, start)

	buoy := buoys[0]
	newPosition(context, start, buoy)

	duration := 0.0
	initNow := startTime.UTC()
	now := startTime.UTC()
	location, err := time.LoadLocation("Europe/Paris")
	if err == nil {
		startTime = startTime.In(location)
	}
	w, w1, x := context.winds.FindWinds(now)

	nav := make(map[int]*Position)
	reached := false
	navDuration := 0.0

	dist := context.DistanceTo(start, buoy.destination())
	if dist/1000.0 < 1000.0 {
		context.maxDistFactor = 1.5
	}
	if dist/1000.0 < 100.0 {
		context.maxDistFactor = 2.0
	}

	if context.isExpes("sqrt-dist-from") {
		dist = cartesian.DistanceTo(start, buoy.destination())
	}

	pos := context.positionProvider.get()
	pos.Latlon = start
	pos.bearing = bearing
	pos.sail = currentSail
	pos.distTo = dist

	wb, _ := wind.Interpolate(w, w1, pos.Latlon.Lat, pos.Latlon.Lon, x)
	pos.twa = float64(bearing) - wb
	if pos.twa < -180 {
		pos.twa += 360
	}
	if pos.twa > 180 {
		pos.twa = pos.twa - 360
	}
	az := int(float64(bearing) * buoy.getFactor())
	nav[az] = pos

	max := make(map[int]float64, int(float64(360)*buoy.getFactor()))
	maxMax := 0.0
	min := dist

	result := Navs{
		Navs: make([]Nav, 0, 1)}
	currentNav := 0
	currentIso := 0
	result.Navs = append(result.Navs, Nav{buoy.name(), make([]Isochrone, 0, int(maxDuration/context.delta))})
	result.Navs[currentNav].Isochrones = append(result.Navs[currentNav].Isochrones, Isochrone{"ff0000", [][]IsochronePosition{[]IsochronePosition{pos.forIsochrone()}}})

	success := true

	var previousDoorIsochrones []map[int]*Position
	var previousDoorFactor float64

	isochrones := make([]map[int]*Position, 0, int(maxDuration/context.delta))
	mustStop := false
	for ok := true; ok; ok = duration < maxDuration && len(buoys) > 0 && !mustStop {
		d := context.delta
		if context.isExpes("progressive-intervales") {

			ks := make([]int, len(deltas))
			kj := 0
			for k := range deltas {
				ks[kj] = k
				kj++
			}
			sort.Ints(ks)
			for _, ke := range ks {
				if duration < float64(ke) {
					d = deltas[ke]
					break
				}
			}
		}
		// if duration < 3 && d > 1 {
		// 	d = 1.0
		// }
		// if duration < 48 && d > 1 {
		// 	d = 1.0
		// }

		maxMax = 0.0
		for _, dist := range max {
			if maxMax < dist {
				maxMax = dist
			}
		}

		nextIsochrone := make(map[int]*Position)
		if len(previousDoorIsochrones) > 0 {
			for _, p := range previousDoorIsochrones[0] {
				if p.duration <= duration+d {
					for az, pdi := range previousDoorIsochrones[0] {
						nextIsochrone[int(float64(az)/previousDoorFactor*buoy.getFactor())] = pdi
					}
					log.Debugf("nextIsochrone : %d - %f %f\n", len(nextIsochrone), duration, p.duration)
					d = p.duration - duration
					previousDoorIsochrones = previousDoorIsochrones[1:len(previousDoorIsochrones)]
				}
				break
			}
		}

		log.Debugf("Nav(%s) %f:%.1f - %d(%d) to %s (%.1f km)\n", buoy.name(), duration, d, len(nav), int(buoy.getFactor()), buoy.name(), min/1000.0)
		nav, reached, navDuration = navigate(&context, now, buoy.getFactor(), max, &min, pos, nav, nextIsochrone, buoy, d)
		log.Debugf("NavDuration : %.1f - %d (%t)\n", navDuration, len(nav), reached)

		duration += navDuration

		keys := make([]int, len(nav))
		j := 0
		for k := range nav {
			keys[j] = k
			j++
		}
		sort.Ints(keys)

		r1 := math.Abs((duration + 0.00001) - math.Floor((duration+0.00001)/1.0)*1.0)
		r6 := math.Abs((duration + 0.00001) - math.Floor((duration+0.00001)/6.0)*6.0)
		r24 := math.Abs((duration + 0.00001) - math.Floor((duration+0.00001)/24.0)*24.0)

		if navDuration >= 1 || r1 < navDuration {
			color := "#cc8dfc"
			if r24 < navDuration {
				color = "#8dfccc"
			} else if r6 < navDuration {
				color = "#fccc8d"
			}
			result.Navs[currentNav].Isochrones = append(result.Navs[currentNav].Isochrones, Isochrone{color, make([][]IsochronePosition, 0, int(maxDuration/context.delta))})
			currentIso = currentIso + 1

			var navSlice []IsochronePosition
			previousK := -99
			for _, k := range keys {
				if k-previousK > 6 {
					if previousK > 0 {
						result.Navs[currentNav].Isochrones[currentIso].Paths = append(result.Navs[currentNav].Isochrones[currentIso].Paths, navSlice)
					}
					navSlice = make([]IsochronePosition, 0, len(nav))
				}
				//		fmt.Println(int(duration), k, nav[k])
				navSlice = append(navSlice, nav[k].forIsochrone())
				previousK = k
			}
			result.Navs[currentNav].Isochrones[currentIso].Paths = append(result.Navs[currentNav].Isochrones[currentIso].Paths, navSlice)
		}

		if !reached && len(nav) == 0 && len(previousDoorIsochrones) == 0 {
			log.Debug("No way found")
			success = false
			break
		}
		isochrones = append(isochrones, nav)

		now = now.Add(time.Duration(int(navDuration*60.0)) * time.Minute)
		if reached {
			if buoy.buoyType() == "WAYPOINT" {
				log.Debugf("Waypoint %s reached %dj %.1fh\n", buoy.name(), int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration))
				if stop {
					mustStop = true
				}
			} else if buoy.buoyType() == "DOOR" {
				d := buoy.(*Door)
				log.Debugf("Door %s reached %dj %.1fh\n", buoy.name(), int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration))

				if len(d.reachers.isochrones) == 0 {
					log.Debug("No way found")
					success = false
					break
				}
				nav = d.reachers.isochrones[0]
				for _, n := range nav {
					now = initNow.Add(time.Duration(int(n.duration*60.0)) * time.Minute)
					duration = n.duration
					break
				}
				previousDoorIsochrones = d.reachers.isochrones[1:len(d.reachers.isochrones)]
				previousDoorFactor = d.getFactor()
			}
			buoys = buoys[1:]
			if len(buoys) > 0 {
				pos = newPos(buoy.departure())
				buoy = buoys[0]
				if context.isExpes("sqrt-dist-from") {
					dist = cartesian.DistanceTo(pos.Latlon, buoy.destination())
				} else {
					dist = context.DistanceTo(pos.Latlon, buoy.destination())
				}
				pos.distTo = dist
				min = newPosition(context, pos.Latlon, buoy)

				max = make(map[int]float64, 360*int(buoy.getFactor()))
				// previousFactor = 1
				result.Navs = append(result.Navs, Nav{buoy.name(), make([]Isochrone, 0, int(maxDuration/context.delta))})
				currentNav++
				currentIso = -1
			}
		}
	}

	var last *Position
	var minDist float64

	sails := make(map[byte]float64, 7)
	sails[byte(0)] = 0
	sails[byte(1)] = 0
	sails[byte(2)] = 0
	sails[byte(3)] = 0
	sails[byte(4)] = 0
	sails[byte(5)] = 0
	sails[byte(6)] = 0
	foils := 0.0

	for iso := len(isochrones) - 1; iso >= 0; iso-- {
		first := true
		for ipt, pt := range isochrones[iso] {
			if first || pt.distTo < minDist {
				minDist = pt.distTo
				first = false
				last = isochrones[iso][ipt]
			}
		}
		if !first {
			break
		}
	}

	next := last
	result.WindLine = make([]WindLinePosition, 0, int(maxDuration/context.delta))
	result.WindLine = append(result.WindLine, WindLinePosition{
		Lat:       last.Latlon.Lat,
		Lon:       last.Latlon.Lon,
		Twa:       0,
		Bearing:   0,
		Wind:      0,
		WindSpeed: 0.0,
		BoatSpeed: 0.0,
		Duration:  last.duration,
		Change:    false})
	sails[last.sail] += last.navDuration
	if last.foil > 0 {
		foils += last.navDuration
	}
	for ok := last.previousWindLine != nil; ok; ok = last.previousWindLine != nil {
		last = last.previousWindLine
		sails[last.sail] += last.navDuration
		if last.foil > 1 {
			foils += last.navDuration
		}
		result.WindLine = append(result.WindLine, WindLinePosition{
			Lat:       last.Latlon.Lat,
			Lon:       last.Latlon.Lon,
			Twa:       next.twa,
			Bearing:   next.bearing,
			Wind:      next.wind,
			WindSpeed: next.windSpeed,
			BoatSpeed: next.boatSpeed,
			Sail:      next.sail,
			Foil:      next.foil,
			Ice:       next.isInIceLimits,
			Duration:  last.duration,
			Change:    next.change})
		next = last
	}

	result.Sumup = Sumup{
		Start:         startTime,
		Duration:      duration,
		SailsDuration: sails,
		FoilDuration:  foils,
		Success:       success}

	msg := fmt.Sprintf("%s : %dj %.1fh - %d L %d H %d C0\n", startTime.Format("02 Jan 15:04"), int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration), int(sails[byte(3)]+sails[byte(6)]), int(sails[byte(2)]+sails[byte(5)]), int(sails[byte(4)]))
	xm.Send(msg)

	for iso := 0; iso < len(isochrones); iso++ {
		for _, pt := range isochrones[iso] {
			context.positionProvider.put(pt)
		}
	}

	return result
}

func RunTestIsLand(l *land.Land) Navs {
	res := make([][]IsochronePosition, 0, 1800)
	//for lat := 48.0 ; lat < 49.0 ; lat+=0.001 {
	for lat := -90.0; lat < 90.0; lat += 1.0 {
		partial := make([]IsochronePosition, 0, 3600)
		previousIsLand := true
		//for lon := -5.0 ; lon < -4.0 ; lon+=0.001 {
		for lon := -180.0; lon < 180.0; lon += 1.0 {
			isLand := l.IsLand(float64(lat), float64(lon))
			if !isLand && previousIsLand {
				if len(partial) > 0 {
					res = append(res, partial)
				}
				partial = make([]IsochronePosition, 0, 360)
			}
			if !isLand {
				partial = append(partial, IsochronePosition{Latlon: latlon.LatLon{Lat: float64(lat), Lon: float64(lon)}})
			}
			previousIsLand = isLand
		}
		if len(partial) > 0 {
			res = append(res, partial)
		}
	}
	isochrone := Isochrone{Color: "#ff0000", Paths: res}
	nav := Nav{Isochrones: []Isochrone{isochrone}}
	navs := Navs{Navs: []Nav{nav}}
	return navs
}
