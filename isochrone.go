package main

import (
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"sort"
	"sync"
	"time"

	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
	"github.com/a-bouts/nav-server/xmpp"
)

type Context struct {
	expes      map[string]bool
	race       Race
	polar      polar.Polar
	boat       polar.Boat
	land       *Land
	winds      map[string][]*wind.Wind
	winchMalus float64
	LatLonSpherical
	maxDistFactor float64
	delta         float64

	positionPool sync.Pool

	newPolars             bool
	progressiveIntervales bool
	sqrtFromDist          bool
	optim                 bool
	maxDist               bool
	alternatives          bool
	vent                  int
}

type Isochrone struct {
	Color string       `json:"color"`
	Paths [][]Position `json:"paths"`
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

type Alternative struct {
	best         int
	alternatives [14]*Position
}

type Position struct {
	Latlon           LatLon `json:"latlon"`
	az               int
	fromDist         float64
	bearing          int
	twa              float64
	wind             float64
	windSpeed        float64
	boatSpeed        float64
	sail             byte
	foil             int
	distTo           float64
	previousWindLine *Position
	duration         float64
	navDuration      float64
	isLand           bool
	bonus            int //0 nothing, 1 line, 2 wind
	change           bool
	reached          bool
	doorReached      string
	isInIceLimits    bool
	fromAlternative  bool
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
	Foil      int     `json:"foil"`
	Ice       bool    `json:"ice"`
	Duration  float64 `json:"duration"`
	Change    bool    `json:"change"`
}

type IsLand struct {
	IsLand bool `json:"island"`
}

func (pos *Position) clear() {
	pos.az = 0
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
	pos.bonus = 0
	pos.change = false
	pos.reached = false
	pos.doorReached = ""
	pos.isInIceLimits = false
	pos.fromAlternative = false
}

func (context *Context) isExpes(expe string) bool {

	if context.expes != nil {
		if _, found := context.expes[expe]; found && context.expes[expe] {
			return true
		}
	}

	return false
}

func isLand2(l *Land, pos Position, c chan Position) {
	//url := fmt.Sprintf("https://wind.bouts.me/land/%f/%f", pos.Latlon.Lat, pos.Latlon.Lon)
	url := fmt.Sprintf("http://land-server:5000/land/%f/%f", pos.Latlon.Lat, pos.Latlon.Lon)
	resp, err := http.Get(url)
	if err != nil {
		fmt.Println(url, err)
		c <- pos
		return
	}
	defer resp.Body.Close()
	var isLand IsLand
	_ = json.NewDecoder(resp.Body).Decode(&isLand)
	pos.isLand = isLand.IsLand
	c <- pos
}

func isLand(l *Land, pos Position, c chan Position) {
	pos.isLand = l.IsLand(pos.Latlon.Lat, pos.Latlon.Lon)
	c <- pos
}

func isToAvoid(buoy Buoy, p LatLon) bool {
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

var cartesian LatLonHaversine = LatLonHaversine{}

func jump(context *Context, start *Position, buoy Buoy, src *Position, b float64, wb float64, ws float64, d float64, factor float64, min *float64, isAlternative bool) (int, *Position) {
	bonus := 0
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
		if isAlternative {
			return 0, nil
		}
		// changement de voile = vitesse / 2 pendant 5 minutes : on enlève 2 minutes et demi
		dist = boatSpeed * 1.852 * (d*60.0 - context.winchMalus/2) / 60 * 1000.0
		change = true

	}

	to := context.Destination(src.Latlon, float64(b), dist)

	if context.land != nil && context.land.IsLand(to.Lat, to.Lon) {
		return 0, nil
	}

	fullDist, az := 0.0, 0.0
	if context.sqrtFromDist {
		fullDist, az = cartesian.DistanceAndBearingTo(start.Latlon, to)
	} else {
		fullDist, az = context.DistanceAndBearingTo(start.Latlon, to)
	}

	res := context.positionPool.Get().(*Position)
	res.clear()
	res.Latlon = to
	res.az = int(math.Round(az * factor))
	res.fromDist = fullDist
	res.bearing = bearing
	res.twa = twa
	res.wind = wb
	res.windSpeed = ws * 1.943844
	res.boatSpeed = boatSpeed
	res.sail = sail
	res.foil = isFoil
	res.isInIceLimits = context.race.IceLimits.isInIceLimits(&to)
	res.bonus = bonus
	res.duration = d + src.duration
	res.navDuration = d
	res.previousWindLine = src
	res.change = change
	res.fromAlternative = isAlternative

	if buoy != nil {
		if context.sqrtFromDist {
			res.distTo = cartesian.DistanceTo(to, buoy.destination())
		} else {
			res.distTo = context.DistanceTo(to, buoy.destination())
		}
		if res.distTo < *min {
			*min = res.distTo
		}
	}
	return res.az, res
}

func doorReached(context *Context, start *Position, src *Position, buoy Buoy, wb float64, ws float64, duration float64, factor float64) (bool, float64, *Position) {
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
		if context.sqrtFromDist {
			fullDist, az = cartesian.DistanceAndBearingTo(start.Latlon, latlon)
		} else {
			fullDist, az = context.DistanceAndBearingTo(start.Latlon, latlon)
		}
		res := context.positionPool.Get().(*Position)
		res.clear()
		res.Latlon = latlon
		res.az = int(math.Round(az * factor))
		res.fromDist = fullDist
		res.bearing = int(math.Round(az12))
		res.twa = twa
		res.wind = wb
		res.windSpeed = ws * 1.943844
		res.boatSpeed = boatSpeed
		res.sail = sail
		res.foil = isFoil
		res.isInIceLimits = context.race.IceLimits.isInIceLimits(&latlon)
		res.bonus = 0
		res.distTo = 0
		res.duration = durationToWaypoint + src.duration
		res.navDuration = durationToWaypoint
		res.previousWindLine = src
		res.change = change

		return true, durationToWaypoint, res
	}
	return false, 0, nil
}

func alternative(position *Position) int {
	if position.twa >= 0 {
		return int(position.sail) // 0 - 6
	} else {
		return 7 + int(position.sail) // 7 - 13
	}

}

func getAlternative(p *Position) *Alternative {
	a := alternative(p)
	alt := Alternative{
		best: a}
	alt.alternatives[a] = p
	return &alt
}

func (alt *Alternative) getBest() *Position {
	return alt.alternatives[alt.best]
}

func way(context *Context, start *Position, src *Position, wb float64, ws float64, duration float64, buoy Buoy, factor float64, min *float64, isAlternative bool) (map[int](*Alternative), bool, float64) {
	result := make(map[int](*Alternative))

	reached, dur, point := doorReached(context, start, src, buoy, wb, ws, duration, factor)
	if reached {
		result[point.az] = getAlternative(point)
		return result, true, dur
	}

	bMin := 0
	bMax := 360
	if context.optim {
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
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min, isAlternative)
			if to != nil {
				prev, exists := result[az]
				a := alternative(to)
				if !exists {
					result[az] = getAlternative(to)
				} else {
					if prev.alternatives[a] == nil || prev.alternatives[a].fromDist < to.fromDist {
						prev.alternatives[a] = to
					}
					if prev.alternatives[prev.best].fromDist < to.fromDist {
						prev.best = a
					}
				}
			}
		}
	} else {
		for b := float64(bMin); b < 360.0; b += 1.0 {
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min, false)
			if to != nil {
				prev, exists := result[az]
				a := alternative(to)
				if !exists {
					result[az] = getAlternative(to)
				} else {
					if prev.alternatives[a] == nil || prev.alternatives[a].fromDist < to.fromDist {
						prev.alternatives[a] = to
					}
					if prev.alternatives[prev.best].fromDist < to.fromDist {
						prev.best = a
					}
				}
			}
		}
		for b := 0.0; b < float64(bMax); b += 1.0 {
			az, to := jump(context, start, buoy, src, b, wb, ws, duration, factor, min, false)
			if to != nil {
				prev, exists := result[az]
				a := alternative(to)
				if !exists {
					result[az] = getAlternative(to)
				} else {
					if prev.alternatives[a] == nil || prev.alternatives[a].fromDist < to.fromDist {
						prev.alternatives[a] = to
					}
					if prev.alternatives[prev.best].fromDist < to.fromDist {
						prev.best = a
					}
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
		reachedResult := make(map[int](*Alternative))
		for az, res := range result {
			alt := res.alternatives[res.best]
			//check if bearing allow to cross the door
			if b < t && (a < b && float64(alt.bearing) > a && float64(alt.bearing) < b || a > b && (float64(alt.bearing) > a || float64(alt.bearing) < b)) {
				a2 := context.BearingTo(alt.Latlon, buoy.(*Door).Left)
				alpha2 := 180 + a2 - t
				if a2 > 180 {
					alpha2 = alpha2 - 360
				}
				b2 := context.BearingTo(alt.Latlon, buoy.(*Door).Right)
				beta2 := b2 - t
				// if b2 > 180 {
				// 	beta2 = beta2 - 360
				// }

				if alpha*alpha2 < 0 && beta*beta2 < 0 {
					//fmt.Println("t", t, "a", a, "alpha", alpha, "a2", a2, "alpha2", alpha2, "b", b, "beta", beta, "b2", b2, "beta2", beta2)
					reachedResult[az] = res
					alt.reached = true
					alt.doorReached = buoy.name()
				}
			}
		}
		if reached {
			return reachedResult, true, duration
		}
	}

	return result, false, duration
}

func navigate(context *Context, now time.Time, factor float64, max map[int]float64, min *float64, start *Position, previous_isochrone map[int]*Alternative, isochrone map[int]*Alternative, buoy Buoy, duration float64) (map[int]*Alternative, bool, float64) {
	reached := false
	var lock = sync.RWMutex{}

	minWayDuration := duration
	var wg sync.WaitGroup
	cpt := 0

	w, w1, x := findWinds(context.winds, now)

	expeAlternatives := context.alternatives
	for _, alt := range previous_isochrone {
		for a, src := range alt.alternatives {
			if !expeAlternatives && a != alt.best || src == nil {
				continue
			}
			isAlternative := a != alt.best

			cpt++
			wg.Add(1)

			go func(src *Position) {

				wb, ws := wind.Interpolate(w, w1, src.Latlon.Lat, src.Latlon.Lon, x, context.vent)

				way, reachedByWay, wayDuration := way(context, start, src, wb, ws, duration, buoy, factor, min, isAlternative)
				if reachedByWay {
					if buoy.buoyType() == "WAYPOINT" {
						for b, dst := range way {
							lock.Lock()
							_, exists := isochrone[b]
							if !exists || len(isochrone) > 1 {
								isochrone = make(map[int]*Alternative)
								exists = false
							}
							if !exists || isochrone[b].getBest().duration > wayDuration {
								isochrone[b] = dst
								minWayDuration = wayDuration
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
						if !exists {
							lock.Lock()
							isochrone[az] = dst
							lock.Unlock()
						} else {
							lock.Lock()
							if context.alternatives {
								for a, alt := range dst.alternatives {
									if alt != nil && (prev.alternatives[a] == nil || prev.alternatives[a].fromDist < alt.fromDist) {
										prev.alternatives[a] = alt
									}
								}
							}
							if prev.getBest().fromDist < dst.getBest().fromDist {
								prev.alternatives[dst.best] = dst.getBest()
								prev.best = dst.best
							}
							lock.Unlock()
						}
					}
				}
				wg.Done()
			}(src)
			if cpt%15 == 0 {
				wg.Wait()
			}
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
		nbAlternatives := 0
		for az, alt := range isochrone {
			dst := alt.alternatives[alt.best]
			parentReached := false
			p := dst
			for i := 0; i < 10; i++ {
				p = p.previousWindLine
				if p == nil {
					break
				}
				if p != nil && p.reached && p.doorReached == buoy.name() {
					parentReached = true
					break
				}
			}

			if dst.reached && dst.doorReached == buoy.name() || parentReached {
				buoy.reach(context, alt)
			}
			d, e := max[az]
			if e && d > dst.fromDist {
				nbLtMax++
				toRemove = append(toRemove, az)
			} else {
				maxDistFactor := context.maxDistFactor
				if context.maxDist {
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
				} else if len(isochrone)-len(toRemove) > 25 && dst.distTo > 5**min {
					nbFarFromMin++
					toRemove = append(toRemove, az)
				} else {
					max[az] = dst.fromDist
					for i := 0; i < 14; i++ {
						if i != alt.best && alt.alternatives[i] != nil && alt.alternatives[i].fromDist < dst.previousWindLine.fromDist {
							alt.alternatives[i] = nil
							nbAlternatives++
						}
					}
				}
			}
		}
		if nbLtMax+nbToAvoid+nbLand+nbFar+nbFarFromMin > 0 {
			fmt.Println("Remove", nbLtMax, "< Max, ", nbToAvoid, "to avoid, ", nbLand, "landi, ", nbFar, "far from way, ", nbFarFromMin, "far from min, ", nbAlternatives, "bad alternatives")
		}
		for _, az := range toRemove {
			delete(isochrone, az)
		}
	}
	return isochrone, reached, minWayDuration
}

func newPos(departure LatLon) *Position {
	return &Position{
		Latlon:   departure,
		fromDist: 0.0}
}

func newPosition(context Context, start LatLon, buoy Buoy) float64 {
	dist, _ := context.DistanceAndBearingTo(start, buoy.destination())
	fmt.Printf("Go to waypoint %s (*%f - %.1f km)\n", buoy.name(), buoy.getFactor(), dist/1000.0)

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

func Run(expes map[string]bool, l *Land, winds map[string][]*wind.Wind, xm *xmpp.Xmpp, start LatLon, bearing int, currentSail byte, race Race, delta float64, deltas map[int]float64, maxDuration float64, delay float64, sail int, foil bool, hull bool, winchMalus float64, stop bool) Navs {

	context := Context{
		expes:         expes,
		race:          race,
		boat:          polar.Boat{Foil: foil, Hull: hull, Sails: byte(sail)},
		land:          l,
		winds:         winds,
		winchMalus:    winchMalus,
		maxDistFactor: 1.5,
		delta:         delta,
		positionPool: sync.Pool{
			New: func() interface{} {
				return new(Position)
			},
		}}

	context.newPolars = context.isExpes("new-polars")
	context.progressiveIntervales = context.isExpes("progressive-intervales")
	context.sqrtFromDist = context.isExpes("sqrt-from-dist")
	context.optim = context.isExpes("optim")
	context.maxDist = context.isExpes("max-dist")
	context.alternatives = context.isExpes("alternatives")

	context.vent = 0
	if context.isExpes("vent1") {
		context.vent = 1
	} else if context.isExpes("vent2") {
		context.vent = 2
	} else if context.isExpes("vent3") {
		context.vent = 3
	} else if context.isExpes("vent4") {
		context.vent = 4
	}

	var z polar.Polar
	z = polar.Init(polar.Options{Race: race.Polars, Sail: sail})

	if context.newPolars {
		fmt.Println("Load new polars")
		z = polar.Load(polar.Options{Race: race.Boat, Sail: sail})
	}

	context.polar = z

	buoys := race.GetBuyos(context, start)

	buoy := buoys[0]
	newPosition(context, start, buoy)

	duration := 0.0
	initNow := time.Now().UTC()
	now := initNow
	now = now.Add(time.Duration(int(math.Round(delay * 60))) * time.Minute)
	startTime := time.Now().Add(time.Duration(int(math.Round(delay * 60))) * time.Minute)
	location, err := time.LoadLocation("Europe/Paris")
	if err == nil {
		startTime = startTime.In(location)
	}
	w, w1, x := findWinds(winds, now)

	nav := make(map[int]*Alternative)
	reached := false
	navDuration := 0.0

	dist := context.DistanceTo(start, buoy.destination())
	if dist/1000.0 < 1000.0 {
		context.maxDistFactor = 1.5
	}
	if dist/1000.0 < 100.0 {
		context.maxDistFactor = 2.0
	}

	if context.sqrtFromDist {
		dist = cartesian.DistanceTo(start, buoy.destination())
	}

	pos := context.positionPool.Get().(*Position)
	pos.clear()
	pos.Latlon = start
	pos.az = int(float64(bearing) * buoy.getFactor())
	pos.bearing = bearing
	pos.sail = currentSail
	pos.distTo = dist
	pos.fromDist = 0.0
	pos.duration = 0.0
	pos.navDuration = 0.0

	wb, _ := wind.Interpolate(w, w1, pos.Latlon.Lat, pos.Latlon.Lon, x, context.vent)
	pos.twa = float64(bearing) - wb
	if pos.twa < -180 {
		pos.twa += 360
	}
	if pos.twa > 180 {
		pos.twa = pos.twa - 360
	}
	nav[pos.az] = getAlternative(pos)

	max := make(map[int]float64, int(float64(360)*buoy.getFactor()))
	maxMax := 0.0
	min := dist

	result := Navs{
		Navs: make([]Nav, 0, 1)}
	currentNav := 0
	currentIso := 0
	result.Navs = append(result.Navs, Nav{buoy.name(), make([]Isochrone, 0, int(maxDuration/delta))})
	result.Navs[currentNav].Isochrones = append(result.Navs[currentNav].Isochrones, Isochrone{"ff0000", [][]Position{[]Position{*pos}}})

	success := true

	var previousDoorIsochrones []map[int]*Alternative
	var previousDoorFactor float64

	isochrones := make([]map[int]*Alternative, 0, int(maxDuration/delta))
	mustStop := false
	for ok := true; ok; ok = duration < maxDuration && len(buoys) > 0 && !mustStop {
		d := delta
		if context.progressiveIntervales {

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

		nextIsochrone := make(map[int]*Alternative)
		if len(previousDoorIsochrones) > 0 {
			for _, p := range previousDoorIsochrones[0] {
				if p.getBest().duration <= duration+d {
					for az, pdi := range previousDoorIsochrones[0] {
						nextIsochrone[int(float64(az)/previousDoorFactor*buoy.getFactor())] = pdi
					}
					fmt.Printf("nextIsochrone : %d - %f %f\n", len(nextIsochrone), duration, p.getBest().duration)
					d = p.getBest().duration - duration
					previousDoorIsochrones = previousDoorIsochrones[1:len(previousDoorIsochrones)]
				}
				break
			}
		}

		fmt.Printf("Nav(%s) %f:%.1f - %d(%d) to %s (%.1f km)\n", buoy.name(), duration, d, len(nav), int(buoy.getFactor()), buoy.name(), min/1000.0)
		nav, reached, navDuration = navigate(&context, now, buoy.getFactor(), max, &min, pos, nav, nextIsochrone, buoy, d)

		nbAlternatives := 0
		for _, na := range nav {
			nbAlternatives--
			for range na.alternatives {
				nbAlternatives++
			}
		}

		fmt.Printf("NavDuration : %.1f - %d(+%d) (%t)\n", navDuration, len(nav), nbAlternatives, reached)

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
			result.Navs[currentNav].Isochrones = append(result.Navs[currentNav].Isochrones, Isochrone{color, make([][]Position, 0, int(maxDuration/delta))})
			currentIso = currentIso + 1

			var navSlice []Position
			previousK := -99
			for _, k := range keys {
				if k-previousK > 6 {
					if previousK > 0 {
						result.Navs[currentNav].Isochrones[currentIso].Paths = append(result.Navs[currentNav].Isochrones[currentIso].Paths, navSlice)
					}
					navSlice = make([]Position, 0, len(nav))
				}
				//		fmt.Println(int(duration), k, nav[k])
				navSlice = append(navSlice, *nav[k].getBest())
				previousK = k
			}
			result.Navs[currentNav].Isochrones[currentIso].Paths = append(result.Navs[currentNav].Isochrones[currentIso].Paths, navSlice)
		}

		if !reached && len(nav) == 0 && len(previousDoorIsochrones) == 0 {
			fmt.Println("No way found")
			success = false
			break
		}
		isochrones = append(isochrones, nav)

		now = now.Add(time.Duration(int(navDuration*60.0)) * time.Minute)
		if reached {
			if buoy.buoyType() == "WAYPOINT" {
				fmt.Printf("Waypoint %s reached %dj %.1fh\n", buoy.name(), int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration))
				if stop {
					mustStop = true
				}
			} else if buoy.buoyType() == "DOOR" {
				d := buoy.(*Door)
				fmt.Printf("Door %s reached %dj %.1fh\n", buoy.name(), int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration))

				if len(d.reachers.isochrones) == 0 {
					fmt.Println("No way found")
					success = false
					break
				}
				nav = d.reachers.isochrones[0]
				for _, n := range nav {
					now = initNow.Add(time.Duration(int(n.getBest().duration*60.0)) * time.Minute)
					duration = n.getBest().duration
					break
				}
				previousDoorIsochrones = d.reachers.isochrones[1:len(d.reachers.isochrones)]
				previousDoorFactor = d.getFactor()
			}
			buoys = buoys[1:]
			if len(buoys) > 0 {
				pos = newPos(buoy.departure())
				buoy = buoys[0]
				if context.sqrtFromDist {
					dist = cartesian.DistanceTo(pos.Latlon, buoy.destination())
				} else {
					dist = context.DistanceTo(pos.Latlon, buoy.destination())
				}
				pos.distTo = dist
				min = newPosition(context, pos.Latlon, buoy)

				max = make(map[int]float64, 360*int(buoy.getFactor()))
				// previousFactor = 1
				result.Navs = append(result.Navs, Nav{buoy.name(), make([]Isochrone, 0, int(maxDuration/delta))})
				currentNav++
				currentIso = -1
			}
		}
	}

	var last Position
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
			if first || pt.getBest().distTo < minDist {
				minDist = pt.getBest().distTo
				first = false
				last = *isochrones[iso][ipt].getBest()
			}
		}
		if !first {
			break
		}
	}

	next := last
	result.WindLine = make([]WindLinePosition, 0, int(maxDuration/delta))
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
		last = *last.previousWindLine
		sails[last.sail] += last.navDuration
		if last.foil > 1 {
			foils += last.navDuration
		}
		if last.fromAlternative {
			fmt.Println("FROM ALTERNATIVE", last.duration)
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

	return result
}

func RunTestIsLand(l *Land) Navs {
	res := make([][]Position, 0, 1800)
	//for lat := 48.0 ; lat < 49.0 ; lat+=0.001 {
	for lat := -90.0; lat < 90.0; lat += 1.0 {
		partial := make([]Position, 0, 3600)
		previousIsLand := true
		//for lon := -5.0 ; lon < -4.0 ; lon+=0.001 {
		for lon := -180.0; lon < 180.0; lon += 1.0 {
			isLand := l.IsLand(float64(lat), float64(lon))
			if !isLand && previousIsLand {
				if len(partial) > 0 {
					res = append(res, partial)
				}
				partial = make([]Position, 0, 360)
			}
			if !isLand {
				partial = append(partial, Position{Latlon: LatLon{Lat: float64(lat), Lon: float64(lon)}})
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
