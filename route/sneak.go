package route

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/a-bouts/nav-server/api/model"
	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
	log "github.com/sirupsen/logrus"
)

type SnakePosition struct {
	Lat       float64 `json:"lat"`
	Lon       float64 `json:"lon"`
	Twa       float64 `json:"t"`
	Bearing   int     `json:"b"`
	Wind      float64 `json:"w"`
	WindSpeed float64 `json:"ws"`
	BoatSpeed float64 `json:"bs"`
	Sail      byte    `json:"s"`
	Foil      uint8   `json:"f"`
	Ice       bool    `json:"i"`
	Duration  float64 `json:"d"`
	Change    bool    `json:"c"`
}

type Sneak struct {
	StartTime time.Time                  `json:"startTime"`
	Bearing   map[int]([]*SnakePosition) `json:"bearing"`
	Twa       map[int]([]*SnakePosition) `json:"twa"`
}

func EvalSneak(route model.Route, winds *wind.Winds, positionPool *sync.Pool) Sneak {

	if route.Params.MaxDuration > 72 {
		route.Params.MaxDuration = 72
	} else if route.Params.MaxDuration == 0 {
		route.Params.MaxDuration = 24
	}

	winchMalus := 5.0
	if route.Options.Winch {
		winchMalus = 1.25
	}

	context := Context{
		route:      route,
		boat:       polar.Boat{Foil: route.Options.Foil, Hull: route.Options.Hull},
		winchMalus: winchMalus,
		positionProvider: positionProviderPool{
			pool: positionPool,
		},
	}

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
	z = polar.Init(polar.Options{Race: context.route.Race.Polars, Sail: context.route.Options.Sail})

	if context.isExpes("new-polars") {
		fmt.Println("Load new polars")
		z = polar.Load(polar.Options{Race: context.route.Race.Boat, Sail: context.route.Options.Sail})
	}

	context.polar = z

	return Sneak{StartTime: route.StartTime,
		Bearing: BearingSneak(&context, winds),
		Twa:     TwaSneak(context, winds),
	}
}

func BearingSneak(context *Context, winds *wind.Winds) map[int]([]*SnakePosition) {
	delta := 1.0

	result := make(map[int]([]*SnakePosition))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)
	log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())

	wb, _ := wind.Interpolate(w, w1, context.route.Start.Lat, context.route.Start.Lon, x, context.vent)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  context.route.Start,
			bearing: b,
			sail:    context.route.CurrentSail}
		pos.twa = wind.Twa(float64(b), wb)
		result[b] = []*SnakePosition{&SnakePosition{
			Lat:      context.route.Start.Lat,
			Lon:      context.route.Start.Lon,
			Twa: 			toFixed(pos.twa, 1),
			Bearing:  b,
			Duration: 0,
		}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < context.route.Params.MaxDuration {

		for b := 0; b < 360; b++ {
			src := result[b][len(result[b])-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x, context.vent)

			_, pos := jump(context, &Position{Latlon: context.route.Start}, nil, hops[b], float64(b), wb, ws, delta, 1, nil, false)
			context.positionProvider.put(hops[b])

			result[b] = append(result[b], &SnakePosition{
				Lat:      pos.Latlon.Lat,
				Lon:      pos.Latlon.Lon,
				Bearing:  b,
				Duration: pos.duration,
			})
			src.Bearing = b
			src.Twa = toFixed(pos.twa, 1)
			src.Wind = toFixed(pos.wind, 1)
			src.WindSpeed = toFixed(pos.windSpeed, 1)
			src.BoatSpeed = toFixed(pos.boatSpeed, 1)
			src.Sail = pos.sail
			src.Foil = pos.foil
			src.Ice = pos.isInIceLimits
			hops[b] = pos
		}

		duration += delta
		now = now.Add(time.Duration(int(delta*60.0)) * time.Minute)
		w, w1, x = winds.FindWinds(now)
		log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())
	}

	return result
}

func TwaSneak(context Context, winds *wind.Winds) map[int]([]*SnakePosition) {
	delta := 1.0

	result := make(map[int]([]*SnakePosition))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)
	log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())

	wb, _ := wind.Interpolate(w, w1, context.route.Start.Lat, context.route.Start.Lon, x, context.vent)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  context.route.Start,
			bearing: b,
			sail:    context.route.CurrentSail}
		pos.twa = wind.Twa(float64(b), wb)
		result[b] = []*SnakePosition{&SnakePosition{
			Lat:      context.route.Start.Lat,
			Lon:      context.route.Start.Lon,
			Twa: 			toFixed(pos.twa, 1),
			Bearing:  b,
			Duration: 0,
		}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < context.route.Params.MaxDuration {

		for b := 0; b < 360; b++ {
			src := result[b][len(result[b])-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x, context.vent)

			var bearing = wind.Heading(src.Twa, wb)

			log.Tracef("TwaJump from (%f, %f) twa %.2f bearing %.2f, wb %.2f, ws %.2f, delta %.1f", hops[b].Latlon.Lat, hops[b].Latlon.Lon, src.Twa, bearing, wb, ws, delta)
			_, pos := jump(&context, &Position{Latlon: context.route.Start}, nil, hops[b], bearing, wb, ws, delta, 1, nil, false)
			context.positionProvider.put(hops[b])

			result[b] = append(result[b], &SnakePosition{
				Lat:      pos.Latlon.Lat,
				Lon:      pos.Latlon.Lon,
				Bearing:  b,
				Duration: pos.duration,
				Twa: 			toFixed(pos.twa, 1),
			})
			src.Bearing = int(bearing)
			src.Wind = toFixed(pos.wind, 1)
			src.WindSpeed = toFixed(pos.windSpeed, 1)
			src.BoatSpeed = toFixed(pos.boatSpeed, 1)
			src.Sail = pos.sail
			src.Foil = pos.foil
			src.Ice = pos.isInIceLimits
			hops[b] = pos
		}

		duration += delta
		now = now.Add(time.Duration(int(delta*60.0)) * time.Minute)
		w, w1, x = winds.FindWinds(now)
		log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())
	}

	return result
}

func toFixed(val float64, n int) float64 {
	mult := math.Pow10(n)
	return math.Round(val * mult) / mult
}
