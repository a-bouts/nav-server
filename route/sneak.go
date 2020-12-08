package route

import (
	"fmt"
	"sync"
	"time"

	"github.com/a-bouts/nav-server/api/model"
	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
	log "github.com/sirupsen/logrus"
)

type Sneak struct {
	StartTime time.Time                     `json:"startTime"`
	Bearing   map[int]([]*WindLinePosition) `json:"bearing"`
	Twa       map[int]([]*WindLinePosition) `json:"twa"`
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

func BearingSneak(context *Context, winds *wind.Winds) map[int]([]*WindLinePosition) {
	delta := 1.0

	result := make(map[int]([]*WindLinePosition))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)
	log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())

	wb, _ := wind.Interpolate(w, w1, context.route.Start.Lat, context.route.Start.Lon, x)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  context.route.Start,
			bearing: b,
			sail:    context.route.CurrentSail}
		pos.twa = float64(b) - wb
		if pos.twa < -180 {
			pos.twa += 360
		}
		if pos.twa > 180 {
			pos.twa = pos.twa - 360
		}
		result[b] = []*WindLinePosition{&WindLinePosition{
			Lat:      context.route.Start.Lat,
			Lon:      context.route.Start.Lon,
			Twa:      pos.twa,
			Bearing:  b,
			Duration: 0,
		}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < context.route.Params.MaxDuration {

		for b := 0; b < 360; b++ {
			src := result[b][len(result[b])-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x)

			_, pos := jump(context, &Position{Latlon: context.route.Start}, nil, hops[b], float64(b), wb, ws, delta, 1, nil)
			context.positionProvider.put(hops[b])

			result[b] = append(result[b], &WindLinePosition{
				Lat:      pos.Latlon.Lat,
				Lon:      pos.Latlon.Lon,
				Bearing:  b,
				Duration: pos.duration,
			})
			src.Bearing = b
			src.Twa = pos.twa
			src.Wind = pos.wind
			src.WindSpeed = pos.windSpeed
			src.BoatSpeed = pos.boatSpeed
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

func TwaSneak(context Context, winds *wind.Winds) map[int]([]*WindLinePosition) {
	delta := 1.0

	result := make(map[int]([]*WindLinePosition))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)
	log.Tracef("Found winds %s : %s - %.2f - %s", now.Format(time.RFC3339), w.String(), x, w1.String())

	wb, _ := wind.Interpolate(w, w1, context.route.Start.Lat, context.route.Start.Lon, x)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  context.route.Start,
			bearing: b,
			sail:    context.route.CurrentSail}
		pos.twa = float64(b) - wb
		if pos.twa < -180 {
			pos.twa += 360
		}
		if pos.twa > 180 {
			pos.twa = pos.twa - 360
		}
		result[b] = []*WindLinePosition{&WindLinePosition{
			Lat:      context.route.Start.Lat,
			Lon:      context.route.Start.Lon,
			Twa:      pos.twa,
			Bearing:  b,
			Duration: 0,
		}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < context.route.Params.MaxDuration {

		for b := 0; b < 360; b++ {
			src := result[b][len(result[b])-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x)

			var bearing = src.Twa + wb
			if bearing > 360 {
				bearing -= 360
			}

			log.Tracef("TwaJump from (%f, %f) twa %.2f bearing %.2f, wb %.2f, ws %.2f, delta %.1f", hops[b].Latlon.Lat, hops[b].Latlon.Lon, src.Twa, bearing, wb, ws, delta)
			_, pos := jump(&context, &Position{Latlon: context.route.Start}, nil, hops[b], bearing, wb, ws, delta, 1, nil)
			context.positionProvider.put(hops[b])

			result[b] = append(result[b], &WindLinePosition{
				Lat:      pos.Latlon.Lat,
				Lon:      pos.Latlon.Lon,
				Bearing:  b,
				Duration: pos.duration,
				Twa:      pos.twa,
			})
			src.Bearing = int(bearing)
			src.Wind = pos.wind
			src.WindSpeed = pos.windSpeed
			src.BoatSpeed = pos.boatSpeed
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
