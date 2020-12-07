package route

import (
	"fmt"
	"sync"
	"time"

	"github.com/a-bouts/nav-server/api/model"
	"github.com/a-bouts/nav-server/latlon"
	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
)

type BoatLine struct {
	Twa  float64         `json:"twa"`
	Line []latlon.LatLon `json:"line"`
}

func GetBoatLines(route model.Route, winds *wind.Winds, positionPool *sync.Pool) []map[int](*BoatLine) {

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

	return []map[int](*BoatLine){BearingLine(&context, winds), TwaLine(context, winds)}
}

func BearingLine(context *Context, winds *wind.Winds) map[int](*BoatLine) {
	delta := 1.0

	result := make(map[int](*BoatLine))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)

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
		result[b] = &BoatLine{Twa: pos.twa, Line: []latlon.LatLon{context.route.Start}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < 72.0 {

		for b := 0; b < 360; b++ {
			src := result[b].Line[len(result[b].Line)-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x)

			_, pos := jump(context, &Position{Latlon: context.route.Start}, nil, hops[b], float64(b), wb, ws, delta, 1, nil)
			context.positionProvider.put(hops[b])

			result[b].Line = append(result[b].Line, pos.Latlon)
			hops[b] = pos
		}

		duration += delta
		now = now.Add(time.Duration(int(delta*60.0)) * time.Minute)
		w, w1, x = winds.FindWinds(now)
	}

	return result
}

func TwaLine(context Context, winds *wind.Winds) map[int](*BoatLine) {
	delta := 1.0

	result := make(map[int](*BoatLine))
	var hops [360]*Position

	now := context.route.StartTime.UTC()

	w, w1, x := winds.FindWinds(now)

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
		result[b] = &BoatLine{Twa: pos.twa, Line: []latlon.LatLon{context.route.Start}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < 72.0 {

		for b := 0; b < 360; b++ {
			src := result[b].Line[len(result[b].Line)-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x)

			var bearing = result[b].Twa + wb
			if bearing > 360 {
				bearing -= 360
			}

			_, pos := jump(&context, &Position{Latlon: context.route.Start}, nil, hops[b], bearing, wb, ws, delta, 1, nil)
			context.positionProvider.put(hops[b])

			result[b].Line = append(result[b].Line, pos.Latlon)
			hops[b] = pos
		}

		duration += delta
		now = now.Add(time.Duration(int(delta*60.0)) * time.Minute)
		w, w1, x = winds.FindWinds(now)
	}

	return result
}
