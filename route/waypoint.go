package route

import (
	"math"

	"github.com/a-bouts/nav-server/latlon"
)

type Door struct {
	Id          uint8
	Name        string
	Destination latlon.LatLon
	Departure   latlon.LatLon
	ToAvoid     [][][]float64
	Left        latlon.LatLon
	Right       latlon.LatLon
	factor      float64
	reachers    *Reachers
}

type Reachers struct {
	isochrones []map[int]*Alternative
}

type Waypoint struct {
	Id          uint8
	Name        string
	Destination latlon.LatLon
	ToAvoid     [][][]float64
	Radius      int
	factor      float64
}

type Buoy interface {
	buoyType() string
	id() uint8
	name() string
	destination() latlon.LatLon
	departure() latlon.LatLon
	toAvoid() [][][]float64
	radius() int
	getFactor() float64
	setFactor(float64)
	reach(context *Context, alt *Alternative)
}

func GetBuyos(context Context) []Buoy {
	var buoys []Buoy
	for id, w := range context.route.Race.Waypoints {
		if w.Validated {
			continue
		}

		if len(w.Latlons) == 1 {
			buoys = append(buoys, &Waypoint{
				Id:          uint8(id),
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
				Id:          uint8(id),
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
		from := context.route.Start
		if i > 0 {
			from = buoys[i-1].departure()
		}
		dist := context.DistanceTo(from, b.destination())
		boatSpeed, _, _ := context.polar.GetBoatSpeed(90, 10.0, context.boat, false)
		distBetweenPoints := boatSpeed * 1.852 * context.delta * 1000.0
		//factor := 1.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))
		factor := 3.0 + math.Round((math.Pi/180.0)/math.Asin(distBetweenPoints/dist))

		if math.IsNaN(factor) {
			factor = 1
		}
		b.setFactor(factor)
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

func (wp Waypoint) id() uint8 {
	return wp.Id
}

func (d Door) id() uint8 {
	return d.Id
}

func (wp Waypoint) destination() latlon.LatLon {
	return wp.Destination
}

func (d Door) destination() latlon.LatLon {
	return d.Destination
}

func (wp Waypoint) departure() latlon.LatLon {
	return wp.Destination
}

func (d Door) departure() latlon.LatLon {
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
		p := &Position{
			Latlon:           pos.Latlon,
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
			change:           pos.change,
			reached:          false}
		last[a].alternatives[alt.best] = p
	}
}
