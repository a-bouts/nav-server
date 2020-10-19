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
	isochrones []map[int]Position
}

type Waypoint struct {
	Name        string
	Destination LatLon
	ToAvoid     [][][]float64
	factor      float64
}

type Buoy interface {
	buoyType() string
	name() string
	destination() LatLon
	departure() LatLon
	toAvoid() [][][]float64
	getFactor() float64
	setFactor(float64)
	reach(context Context, pos *Position)
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
				ToAvoid:     w.ToAvoid})
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
		boatSpeed, _, _ := context.polar.GetBoatSpeed(90, 10.0, context.boat)
		distBetweenPoints := boatSpeed * 1.852 * context.delta * 1000.0
		factor := 1.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))
		if context.experiment {
			factor = 3.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))
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

func (wp *Waypoint) reach(context Context, pos *Position) {
}

func (d *Door) reach(context Context, pos *Position) {

	dist, az := context.DistanceAndBearingTo(d.departure(), pos.Latlon)

	var last map[int]Position

	if len(d.reachers.isochrones) > 0 {
		last = d.reachers.isochrones[len(d.reachers.isochrones)-1]

		for _, v := range last {
			if v.duration != pos.duration {
				last = make(map[int]Position)
				d.reachers.isochrones = append(d.reachers.isochrones, last)
			}
			break
		}
	} else {
		last = make(map[int]Position)
		d.reachers.isochrones = append(d.reachers.isochrones, last)
	}

	a := int(math.Round(az * d.getFactor()))
	_, found := last[a]
	if !found || last[a].fromDist < dist {
		last[a] = Position{
			Latlon:           pos.Latlon,
			az:               a,
			fromDist:         dist,
			bearing:          pos.bearing,
			twa:              pos.twa,
			wind:             pos.wind,
			windSpeed:        pos.windSpeed,
			boatSpeed:        pos.boatSpeed,
			sail:             pos.sail,
			foil:             pos.foil,
			distTo:           0,
			previousWindLine: pos.previousWindLine,
			duration:         pos.duration,
			navDuration:      pos.navDuration,
			isLand:           pos.isLand,
			bonus:            pos.bonus,
			change:           pos.change,
			reached:          false}
	}
}
