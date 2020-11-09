package main

import (
	"fmt"
	"math"
)

type Door struct {
	Name        string
	Destination LatLon
	Departure   LatLon
	ToAvoid     [][][]float64
	Left        LatLon
	Right       LatLon
	factor      float64
	reachers    *Reachers
}

type Reachers struct {
	isochrones []map[int]*Alternative
}

type Waypoint struct {
	Name        string
	Destination LatLon
	ToAvoid     [][][]float64
	Radius      int
	factor      float64
}

type Buoy interface {
	buoyType() string
	name() string
	destination() LatLon
	departure() LatLon
	toAvoid() [][][]float64
	radius() int
	getFactor() float64
	setFactor(float64)
	reach(context *Context, alt *Alternative)
}

func (race *Race) GetBuyos(context Context, start LatLon) []Buoy {
	var buoys []Buoy
	for _, w := range race.Waypoints {
		if w.Validated {
			continue
		}

		if len(w.Latlons) == 1 {
			buoys = append(buoys, &Waypoint{
				Name:        w.Name,
				Destination: w.Latlons[0],
				ToAvoid:     w.ToAvoid,
				Radius:      w.Radius})
		} else {
			d, t := context.DistanceAndBearingTo(w.Latlons[0], w.Latlons[1])
			a := t - 45
			if a < 0 {
				a += 360
			}
			destination := context.Destination(w.Latlons[0], a, math.Sqrt(d*d/2))
			a = t + 45
			if a > 360 {
				a -= 360
			}
			departure := context.Destination(w.Latlons[0], a, math.Sqrt(d*d/2))

			buoys = append(buoys, &Door{
				Name:        w.Name,
				Destination: destination,
				Departure:   departure,
				ToAvoid:     w.ToAvoid,
				Left:        w.Latlons[0],
				Right:       w.Latlons[1],
				reachers:    &Reachers{}})
		}
	}

	for i, b := range buoys {
		from := start
		if i > 0 {
			from = buoys[i-1].departure()
		}
		dist := context.DistanceTo(from, b.destination())
		boatSpeed, _, _ := context.polar.GetBoatSpeed(90, 10.0, context.boat, false)
		distBetweenPoints := boatSpeed * 1.852 * context.delta * 1000.0
		factor := 1.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))
		if context.progressiveIntervales {
			factor = 1.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))
		}
		if math.IsNaN(factor) {
			factor = 1
		}
		b.setFactor(factor)
	}

	for _, b := range buoys {
		fmt.Println("Buoy", b)
	}

	return buoys
}

func (wp Waypoint) buoyType() string {
	return "WAYPOINT"
}

func (d Door) buoyType() string {
	return "DOOR"
}

func (wp Waypoint) name() string {
	return wp.Name
}

func (d Door) name() string {
	return d.Name
}

func (wp Waypoint) destination() LatLon {
	return wp.Destination
}

func (d Door) destination() LatLon {
	return d.Destination
}

func (wp Waypoint) departure() LatLon {
	return wp.Destination
}

func (d Door) departure() LatLon {
	return d.Departure
}

func (wp Waypoint) toAvoid() [][][]float64 {
	return wp.ToAvoid
}

func (d Door) toAvoid() [][][]float64 {
	return d.ToAvoid
}

func (wp Waypoint) radius() int {
	return wp.Radius
}

func (d Door) radius() int {
	return 0
}

func (wp *Waypoint) setFactor(factor float64) {
	wp.factor = factor
}

func (d *Door) setFactor(factor float64) {
	d.factor = factor
}

func (wp Waypoint) getFactor() float64 {
	return wp.factor
}

func (d Door) getFactor() float64 {
	return d.factor
}

func (wp *Waypoint) reach(context *Context, alt *Alternative) {
}

func (d *Door) reach(context *Context, alt *Alternative) {
	pos := alt.getBest()

	dist, az := context.DistanceAndBearingTo(d.departure(), pos.Latlon)

	var last map[int]*Alternative

	if len(d.reachers.isochrones) > 0 {
		last = d.reachers.isochrones[len(d.reachers.isochrones)-1]

		for _, v := range last {
			if v.getBest().duration != pos.duration {
				last = make(map[int]*Alternative)
				d.reachers.isochrones = append(d.reachers.isochrones, last)
			}
			break
		}
	} else {
		last = make(map[int]*Alternative)
		d.reachers.isochrones = append(d.reachers.isochrones, last)
	}

	a := int(math.Round(az * d.getFactor()))
	_, found := last[a]
	if !found || last[a].getBest().fromDist < dist {
		last[a] = &Alternative{
			best: alt.best}
		p := context.positionPool.Get().(*Position)
		p.clear()

		last[a].alternatives[alt.best] = p
		p.Latlon = pos.Latlon
		p.az = a
		p.fromDist = dist
		p.bearing = pos.bearing
		p.twa = pos.twa
		p.wind = pos.wind
		p.windSpeed = pos.windSpeed
		p.boatSpeed = pos.boatSpeed
		p.sail = pos.sail
		p.foil = pos.foil
		p.distTo = 0
		p.previousWindLine = pos.previousWindLine
		p.duration = pos.duration
		p.navDuration = pos.navDuration
		p.isLand = pos.isLand
		p.bonus = pos.bonus
		p.change = pos.change
		p.reached = false
	}
}
