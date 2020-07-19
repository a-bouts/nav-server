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
	Success       bool             `json:"success"`
}

type Position struct {
	Latlon           LatLon `json:"latlon"`
	az               int
	fromDist         float64
	bearing          int
	twa              int
	wind             int
	windSpeed        float64
	boatSpeed        float64
	sail             byte
	distTo           float64
	previousWindLine *Position
	duration         float64
	navDuration      float64
	isLand           bool
	bonus            int //0 nothing, 1 line, 2 wind
	change           bool
}

type WindLinePosition struct {
	Lat       float64 `json:"lat"`
	Lon       float64 `json:"lon"`
	Twa       int     `json:"twa"`
	Bearing   int     `json:"bearing"`
	Wind      int     `json:"wind"`
	WindSpeed float64 `json:"windSpeed"`
	BoatSpeed float64 `json:"boatSpeed"`
	Sail      byte    `json:"sail"`
	Duration  float64 `json:"duration"`
	Change    bool    `json:"change"`
}

type IsLand struct {
	IsLand bool `json:"island"`
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

func isToAvoid(waypoint Waypoint, p LatLon) bool {
	for _, t := range waypoint.ToAvoid {
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

func jump(z *polar.Polar, boat polar.Boat, start *Position, waypoint *Waypoint, src Position, b float64, wb float64, ws float64, d float64, factor float64, min *float64, winchMalus float64, malus float64) (int, *Position) {
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
	//if diff > 130 || math.Abs(twa) < 30 || math.Abs(twa) > 170 {
	//    return 0, nil
	//}

	bearing := int(math.Round(b))
	t := int(math.Round(twa))

	boatSpeed, sail := z.GetOptimBoatSpeed(twa, ws*3.6, boat, src.sail, winchMalus)
	//if boatSpeed <= 0.0 {
	//    return 0, nil
	//}
	dist := boatSpeed * 1.852 * d * 1000.0
	if int(math.Round(twa))*src.twa < 0 || sail != src.sail {
		// changement de voile = vitesse / 2 pendant 5 minutes : on enlève 2 minutes et demi
		dist = boatSpeed * 1.852 * (d*60.0 - winchMalus/2) / 60 * 1000.0
		change = true
		//if(int(math.Round(twa)) * src.twa >= 0) {
		//		boatSpeed2, _ := z.GetBoatSpeed2(twa, ws * 3.6, boat, int(src.sail))
		//                dist2 := boatSpeed2 * 1.852 * d * 1000.0
		//		if(dist2 > dist) {
		//		    dist = dist2
		//		    sail = src.sail
		//		    change = false
		//		}
		//	    }
	} else {

		//apply malus to simplify navigation
		if bearing == src.bearing {
			bonus = 1
		}
		if t == src.twa {
			bonus = 2
		}
		if bonus == 0 || bonus != src.bonus {
			dist *= malus
		}
	}

	to := src.Latlon.rhumbDestinationPoint(float64(b), dist)

	fullDist, az := start.Latlon.rhumbDistanceAndBearingTo(to)
	distTo := to.rhumbDistanceTo(waypoint.Latlons[0])
	res := Position{
		Latlon:           to,
		az:               int(math.Round(az)),
		fromDist:         fullDist,
		bearing:          bearing,
		twa:              t,
		wind:             int(math.Round(wb)),
		windSpeed:        ws * 1.943844,
		boatSpeed:        boatSpeed,
		sail:             sail,
		bonus:            bonus,
		distTo:           distTo,
		duration:         d + src.duration,
		navDuration:      d,
		previousWindLine: &src,
		change:           change}

	if res.distTo < *min {
		*min = res.distTo
	}
	return int(math.Round(az * factor)), &res
}

func doorReached(start *Position, src Position, waypoint *Waypoint, z *polar.Polar, boat polar.Boat, wb float64, ws float64, duration float64, winchMalus float64) (bool, float64, *Position) {
	distToWaypoint, az12 := src.Latlon.rhumbDistanceAndBearingTo(waypoint.Latlons[0])
	twa := az12 - wb
	if twa < -180 {
		twa += 360
	}
	if twa > 180 {
		twa = 360 - twa
	}
	boatSpeed, sail := z.GetBoatSpeed(twa, ws*3.6, boat)
	durationToWaypoint := (distToWaypoint / 1000.0) / (boatSpeed * 1.852)

	change := false
	if int(math.Round(twa))*src.twa < 0 || sail != src.sail {
		change = true
		//dist := (boatSpeed / 2.0) * 1.852 * 5.0 / 60.0 * 1000.0
		dist := (boatSpeed / 2.0) * 1.852 * winchMalus / 60.0 * 1000.0
		if change {
			if dist > distToWaypoint {
				durationToWaypoint = (distToWaypoint / 1000.0) / ((boatSpeed / 2.0) * 1.852)
			} else {
				durationToWaypoint = 5.0/60.0 + ((distToWaypoint-dist)/1000.0)/(boatSpeed*1.852)
			}
		}
	}

	if durationToWaypoint <= duration {
		fullDist, az := start.Latlon.rhumbDistanceAndBearingTo(waypoint.Latlons[0])
		res := Position{
			Latlon:           waypoint.Latlons[0],
			az:               int(math.Round(az)),
			fromDist:         fullDist,
			bearing:          int(math.Round(az12)),
			twa:              int(math.Round(twa)),
			wind:             int(math.Round(wb)),
			windSpeed:        ws * 1.943844,
			boatSpeed:        boatSpeed,
			sail:             sail,
			bonus:            0,
			distTo:           0,
			duration:         durationToWaypoint + src.duration,
			navDuration:      durationToWaypoint,
			previousWindLine: &src,
			change:           change}

		return true, durationToWaypoint, &res
	}
	return false, 0, nil
}

func way(l *Land, z *polar.Polar, boat polar.Boat, start *Position, src Position, wb float64, ws float64, duration float64, waypoint *Waypoint, isochrone map[int]Position, factor float64, min *float64, winchMalus float64, malus float64) (map[int](*Position), bool, float64) {
	result := make(map[int](*Position))

	if len(waypoint.Latlons) == 1 {
		reached, dur, point := doorReached(start, src, waypoint, z, boat, wb, ws, duration, winchMalus)
		if reached {
			result[point.az] = point
			return result, true, dur
		}
	}

	b_min := 0
	b_max := 360
	// if start.fromDist > 0.0 {
	// 	b_min = start.bearing - 90
	// 	if b_min < 0 {
	// 		b_min = 360 - b_min
	// 	}
	// 	b_max = start.bearing + 90
	// 	if b_max >= 360 {
	// 		b_max = b_max - 360
	// 	}
	// }

	if b_max > b_min {
		for b := float64(b_min); b < float64(b_max); b += 1.0 {
			//for b := 80.0 ; b < 81.0 ; b+=1.0 {
			az, to := jump(z, boat, start, waypoint, src, b, wb, ws, duration, factor, min, winchMalus, malus)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				}
			}
		}
	} else {
		for b := float64(b_min); b < 360.0; b += 1.0 {
			//for b := 80.0 ; b < 81.0 ; b+=1.0 {
			az, to := jump(z, boat, start, waypoint, src, b, wb, ws, duration, factor, min, winchMalus, malus)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				}
			}
		}
		for b := 0.0; b < float64(b_max); b += 1.0 {
			//for b := 80.0 ; b < 81.0 ; b+=1.0 {
			az, to := jump(z, boat, start, waypoint, src, b, wb, ws, duration, factor, min, winchMalus, malus)
			if to != nil {
				prev, exists := result[az]
				if !exists || prev.fromDist < to.fromDist {
					result[az] = to
				}
			}
		}
	}

	// for b := 0.0; b < 360.0; b += 1.0 {
	// 	//for b := 80.0 ; b < 81.0 ; b+=1.0 {
	// 	az, to := jump(z, boat, start, waypoint, src, b, wb, ws, duration, factor, min, winchMalus, malus)
	// 	if to != nil {
	// 		prev, exists := result[az]
	// 		if !exists || prev.fromDist < to.fromDist {
	// 			result[az] = to
	// 		}
	// 	}
	// }

	if len(waypoint.Latlons) == 3 {
		t := waypoint.Latlons[0].rhumbBearingTo(waypoint.Latlons[1])
		a := src.Latlon.rhumbBearingTo(waypoint.Latlons[0])
		alpha := 180 + a - t
		if a > 180 {
			alpha = alpha - 360
		}

		b := src.Latlon.rhumbBearingTo(waypoint.Latlons[1])
		beta := b - t
		if b > 180 {
			beta = beta - 360
		}

		reached := false
		reachedResult := make(map[int](*Position))
		for az, res := range result {
			//check if bearing allow to cross the door
			if b < t && (a < b && float64(res.bearing) > a && float64(res.bearing) < b || a > b && (float64(res.bearing) > a || float64(res.bearing) < b)) {
				a2 := res.Latlon.rhumbBearingTo(waypoint.Latlons[0])
				alpha2 := 180 + a2 - t
				if a2 > 180 {
					alpha2 = alpha2 - 360
				}
				b2 := res.Latlon.rhumbBearingTo(waypoint.Latlons[1])
				beta2 := b2 - t
				if b2 > 180 {
					beta2 = beta2 - 360
				}

				if alpha*alpha2 < 0 && beta*beta2 < 0 {
					fmt.Println("reached")
					fmt.Println("t", t, "a", a, "alpha", alpha, "a2", a2, "alpha2", alpha2, "b", b, "beta", beta, "b2", b2, "beta2", beta2)
					reachedResult[az] = res
					reached = true
				}
			}
		}
		if reached {
			return reachedResult, true, duration
		}
	}

	return result, false, duration
}

func navigate(l *Land, z *polar.Polar, boat polar.Boat, w *wind.Wind, w1 *wind.Wind, x float64, factor float64, max map[int]float64, min *float64, start *Position, previous_isochrone map[int]Position, originalWaypoint Waypoint, waypoint *Waypoint, duration float64, winchMalus float64, malus float64) (map[int]Position, bool, float64) {
	isochrone := make(map[int]Position)
	reached := false
	var lock = sync.RWMutex{}

	minWayDuration := duration
	var wg sync.WaitGroup
	cpt := 0

	for _, src := range previous_isochrone {
		cpt++
		wg.Add(1)

		go func(src Position) {

			wb, ws := w.Interpolate(w1, src.Latlon.Lat, src.Latlon.Lon, x)

			way, reachedByWay, wayDuration := way(l, z, boat, start, src, wb, ws, duration, waypoint, isochrone, factor, min, winchMalus, malus)
			if reachedByWay {
				if len(waypoint.Latlons) == 1 {
					for b, dst := range way {
						lock.Lock()
						_, exists := isochrone[b]
						if !exists || len(isochrone) > 1 {
							isochrone = make(map[int]Position)
						}
						if !exists || isochrone[b].duration > wayDuration {
							isochrone[b] = *dst
							minWayDuration = wayDuration
						}
						lock.Unlock()
					}
				} else {
					if !reached {
						isochrone = make(map[int]Position)
					}
					for az, dst := range way {
						lock.RLock()
						prev, exists := isochrone[az]
						lock.RUnlock()
						if !exists || prev.fromDist < dst.fromDist {
							lock.Lock()
							isochrone[az] = *dst
							lock.Unlock()
						}
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
						isochrone[az] = *dst
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
	wg.Wait()
	toRemove := make([]int, 0, len(isochrone))
	nbLtMax := 0
	nbToAvoid := 0
	nbLand := 0
	nbFar := 0
	nbFarFromMin := 0
	for az, dst := range isochrone {
		d, e := max[az]
		if e && d > dst.fromDist {
			nbLtMax++
			toRemove = append(toRemove, az)
		} else {
			max[az] = dst.fromDist
			if isToAvoid(originalWaypoint, dst.Latlon) {
				nbToAvoid++
				toRemove = append(toRemove, az)
			} else if l.IsLand(dst.Latlon.Lat, dst.Latlon.Lon) {
				nbLand++
				toRemove = append(toRemove, az)
			} else if dst.fromDist+dst.distTo > 1.5*start.distTo {
				nbFar++
				toRemove = append(toRemove, az)
			} else if len(isochrone)-len(toRemove) > 25 && dst.distTo > 5**min {
				nbFarFromMin++
				toRemove = append(toRemove, az)
			}
		}
	}
	if nbLtMax+nbToAvoid+nbLand+nbFar+nbFarFromMin > 0 {
		fmt.Println("Remove", nbLtMax, "< Max, ", nbToAvoid, "to avoid, ", nbLand, "landi, ", nbFar, "far from way, ", nbFarFromMin, "far from min")
	}
	for _, az := range toRemove {
		delete(isochrone, az)
	}
	return isochrone, reached, minWayDuration
}

func newPos(waypoint Waypoint) *Position {
	if len(waypoint.Latlons) == 1 {
		return &Position{
			Latlon:   waypoint.Latlons[0],
			fromDist: 0.0}
	} else {
		return &Position{
			Latlon:   waypoint.Latlons[2],
			fromDist: 0.0}
	}
}

func newPosition(start LatLon, waypoint Waypoint) float64 {
	dist, _ := start.rhumbDistanceAndBearingTo(waypoint.Latlons[0])
	fmt.Printf("Go to waypoint %s (%.1f km)\n", waypoint.Name, dist/1000.0)

	return dist
}

func findWinds(winds map[string]*wind.Wind, m time.Time) (*wind.Wind, *wind.Wind, float64) {

	stamp := m.Format("2006010215")

	keys := make([]string, 0, len(winds))
	for k, _ := range winds {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	if keys[0] >= stamp {
		fmt.Printf("Found winds for '%s' : %s\n", stamp, keys[0])
		return winds[keys[0]], nil, 0
	}
	for i, _ := range keys {
		if keys[i] > stamp {
			h := m.Sub(winds[keys[i-1]].Date).Minutes()
			delta := winds[keys[i]].Date.Sub(winds[keys[i-1]].Date).Minutes()
			fmt.Printf("Found winds for '%s' : %s-%.2f-%s\n", stamp, keys[i-1], h/delta, keys[i])
			return winds[keys[i-1]], winds[keys[i]], h / delta
			//return winds[keys[i-1]], nil, h / delta
		}
	}
	fmt.Printf("Found winds for '%s' : %s\n", stamp, keys[len(keys)-1])
	return winds[keys[len(keys)-1]], nil, 0
}

func Run(l *Land, winds map[string]*wind.Wind, xm *xmpp.Xmpp, start LatLon, bearing int, currentSail byte, race Race, delta float64, maxDuration float64, delay int, sail int, foil bool, hull bool, winchMalus float64, malus float64, stop bool) Navs {
	z := polar.Init(polar.Options{Race: race.Polars, Sail: sail})
	boat := polar.Boat{foil, hull}

	nextWaypoint := 0
	for v := race.IsValidated(nextWaypoint); v; v = race.IsValidated(nextWaypoint) {
		nextWaypoint = race.Reached(nextWaypoint)
	}
	waypoint := race.NextWaypoint(nextWaypoint)
	newPosition(start, waypoint)

	duration := 0.0
	now := time.Now().UTC()
	now = now.Add(time.Duration(delay) * time.Hour)
	startTime := time.Now().Add(time.Duration(delay) * time.Hour)
	location, err := time.LoadLocation("Europe/Paris")
	if err == nil {
		startTime = startTime.In(location)
	}
	w, w1, x := findWinds(winds, now)

	nav := make(map[int]Position)
	reached := false
	navDuration := 0.0

	dist := start.rhumbDistanceTo(race.NextWaypoint(nextWaypoint).Latlons[0])
	pos := &Position{
		Latlon:      start,
		az:          bearing,
		bearing:     bearing,
		sail:        currentSail,
		distTo:      dist,
		fromDist:    0.0,
		duration:    0.0,
		navDuration: 0.0} //float64(delay)}

	wb, _ := w.Interpolate(w1, pos.Latlon.Lat, pos.Latlon.Lon, x)
	pos.twa = int(float64(pos.az) - wb)
	if pos.twa < -180 {
		pos.twa += 360
	}
	if pos.twa > 180 {
		pos.twa = pos.twa - 360
	}
	nav[pos.az] = *pos

	max := make(map[int]float64)
	min := dist

	factor := 0.0

	result := Navs{
		Navs: make([]Nav, 0, 1)}
	currentNav := 0
	currentIso := 0
	result.Navs = append(result.Navs, Nav{waypoint.Name, make([]Isochrone, 0, int(maxDuration/delta))})
	result.Navs[currentNav].Isochrones = append(result.Navs[currentNav].Isochrones, Isochrone{"ff0000", [][]Position{[]Position{*pos}}})

	success := true

	isochrones := make([]map[int]Position, 0, int(maxDuration/delta))
	for ok := true; ok; ok = duration < maxDuration && race.HasNextWaypoint(nextWaypoint) && (!stop || !reached) {
		d := delta
		if duration < 3 && d > 1 {
			d = 1.0
		}
		if duration < 48 && d > 1 {
			d = 1.0
		}

		maxMax := 0.0
		for _, dist := range max {
			if maxMax < dist {
				maxMax = dist
			}
		}
		previousFactor := factor
		factor = 1.0 + math.Round(math.Sin(math.Pi/180.0)*maxMax/15000)
		if factor <= previousFactor {
			factor = previousFactor
		}
		if factor > 0 && factor <= previousFactor && len(nav) < 25 {
			factor += 1
		}

		if factor != previousFactor {
			newMax := make(map[int]float64, 360*int(factor))

			for i := 0; i < 360*int(factor); i++ {
				k := int(float64(i) * previousFactor / factor)
				val, exists := max[k]
				if exists {
					newMax[i] = val
				}
			}
			max = newMax
		}

		fmt.Printf("Nav %f - %d(%d) to %s (%.1f km)\n", duration, len(nav), int(factor), waypoint.Name, min/1000.0)
		nav, reached, navDuration = navigate(l, &z, boat, w, w1, x, factor, max, &min, pos, nav, waypoint, &waypoint, d, winchMalus, malus)

		duration += navDuration

		if len(nav) == 0 {
			fmt.Println("No way found")
			success = false
			break
		}
		isochrones = append(isochrones, nav)
		keys := make([]int, len(nav))
		j := 0
		for k := range nav {
			keys[j] = k
			j++
		}
		sort.Ints(keys)

		color := "#cc8dfc"
		if int(math.Round(duration/delta))%int(24.0/delta) == 0 {
			color = "#8dfccc"
		} else if int(math.Round(duration/delta))%int(6/delta) == 0 {
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
			navSlice = append(navSlice, nav[k])
			previousK = k
		}
		result.Navs[currentNav].Isochrones[currentIso].Paths = append(result.Navs[currentNav].Isochrones[currentIso].Paths, navSlice)

		now = now.Add(time.Duration(int(navDuration*60.0)) * time.Minute)
		w, w1, x = findWinds(winds, now)
		if reached {
			fmt.Printf("Waypoint %s reached %dj %.1fh\n", waypoint.Name, int(duration/24.0), float64(int(duration)%24)+duration-math.Floor(duration))
			nextWaypoint = race.Reached(nextWaypoint)
			max = make(map[int]float64)
			if race.HasNextWaypoint(nextWaypoint) {
				pos = newPos(waypoint)
				waypoint = race.NextWaypoint(nextWaypoint)
				dist := pos.Latlon.rhumbDistanceTo(waypoint.Latlons[0])
				pos.distTo = dist
				min = newPosition(pos.Latlon, waypoint)
				factor = 0
				previousFactor = 1
				result.Navs = append(result.Navs, Nav{waypoint.Name, make([]Isochrone, 0, int(maxDuration/delta))})
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
	fmt.Println(last.distTo)

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
	for ok := last.previousWindLine != nil; ok; ok = last.previousWindLine != nil {
		last = *last.previousWindLine
		sails[last.sail] += last.navDuration
		result.WindLine = append(result.WindLine, WindLinePosition{
			Lat:       last.Latlon.Lat,
			Lon:       last.Latlon.Lon,
			Twa:       next.twa,
			Bearing:   next.bearing,
			Wind:      next.wind,
			WindSpeed: next.windSpeed,
			BoatSpeed: next.boatSpeed,
			Sail:      next.sail,
			Duration:  last.duration,
			Change:    next.change})
		next = last
	}

	result.Sumup = Sumup{
		Start:         startTime,
		Duration:      duration,
		SailsDuration: sails,
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
