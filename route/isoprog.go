package route

import (
	"fmt"
	"sync"
	"time"

	"github.com/a-bouts/nav-server/latlon"
	"github.com/a-bouts/nav-server/polar"
	"github.com/a-bouts/nav-server/wind"
)

type BoatLine struct {
	Twa  float64         `json:"twa"`
	Line []latlon.LatLon `json:"line"`
}

func GetBoatLines(expes map[string]bool, winds *wind.Winds, start latlon.LatLon, bearing int, currentSail byte, race Race, delta float64, startTime time.Time, sail int, foil bool, hull bool, winchMalus float64, positionPool *sync.Pool) []map[int](*BoatLine) {

	context := Context{
		expes:      expes,
		boat:       polar.Boat{Foil: foil, Hull: hull},
		winchMalus: winchMalus,
		//positionProvider: positionProviderNew{},
		positionProvider: positionProviderPool{
			pool: positionPool,
		},
	}

	var z polar.Polar
	z = polar.Init(polar.Options{Race: race.Polars, Sail: sail})

	if context.newPolars {
		fmt.Println("Load new polars")
		z = polar.Load(polar.Options{Race: race.Boat, Sail: sail})
	}

	context.polar = z

	return []map[int](*BoatLine){BearingLine(&context, winds, start, bearing, currentSail, delta, startTime, sail), TwaLine(context, winds, start, bearing, currentSail, delta, startTime, sail)}
}

func BearingLine(context *Context, winds *wind.Winds, start latlon.LatLon, bearing int, currentSail byte, delta float64, startTime time.Time, sail int) map[int](*BoatLine) {

	result := make(map[int](*BoatLine))
	var hops [360]*Position

	now := startTime.UTC()

	w, w1, x := winds.FindWinds(now)

	wb, _ := wind.Interpolate(w, w1, start.Lat, start.Lon, x, 0)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  start,
			bearing: b,
			sail:    currentSail}
		pos.twa = float64(b) - wb
		if pos.twa < -180 {
			pos.twa += 360
		}
		if pos.twa > 180 {
			pos.twa = pos.twa - 360
		}
		result[b] = &BoatLine{Twa: pos.twa, Line: []latlon.LatLon{start}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < 72.0 {

		for b := 0; b < 360; b++ {
			src := result[b].Line[len(result[b].Line)-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x, 0)

			_, pos := jump(context, &Position{Latlon: start}, nil, hops[b], float64(b), wb, ws, delta, 1, nil, false)
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

func TwaLine(context Context, winds *wind.Winds, start latlon.LatLon, bearing int, currentSail byte, delta float64, startTime time.Time, sail int) map[int](*BoatLine) {

	result := make(map[int](*BoatLine))
	var hops [360]*Position

	now := startTime.UTC()

	w, w1, x := winds.FindWinds(now)

	wb, _ := wind.Interpolate(w, w1, start.Lat, start.Lon, x, 0)

	duration := 0.0

	for b := 0; b < 360; b++ {
		pos := Position{
			Latlon:  start,
			bearing: b,
			sail:    currentSail}
		pos.twa = float64(b) - wb
		if pos.twa < -180 {
			pos.twa += 360
		}
		if pos.twa > 180 {
			pos.twa = pos.twa - 360
		}
		result[b] = &BoatLine{Twa: pos.twa, Line: []latlon.LatLon{start}}
		hops[b] = &pos
	}

	for ok := true; ok; ok = duration < 72.0 {

		for b := 0; b < 360; b++ {
			src := result[b].Line[len(result[b].Line)-1]

			wb, ws := wind.Interpolate(w, w1, src.Lat, src.Lon, x, 0)

			var bearing = result[b].Twa + wb
			if bearing > 360 {
				bearing -= 360
			}

			_, pos := jump(&context, &Position{Latlon: start}, nil, hops[b], bearing, wb, ws, delta, 1, nil, false)
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