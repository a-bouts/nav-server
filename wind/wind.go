package wind

import (
	"fmt"
	"math"
	"os"
	"time"

	"github.com/nilsmagnus/grib/griblib"
)

type Wind struct {
	Date time.Time
	File string
	Lat0 float64
	Lon0 float64
	ΔLat float64
	ΔLon float64
	NLat uint32
	NLon uint32
	U    [][]float64
	V    [][]float64
}

type Forecasts struct {
	RefTime       string `json:"refTime"`
	ForecastTimes []int  `json:"forecastTimes"`
}

func roundHours(hours int, interval int) string {
	if interval > 0 {
		result := int(math.Floor(float64(hours)/float64(interval)) * float64(interval))
		return fmt.Sprintf("%02d", result)
	}
	return ""
}

func (w Wind) buildGrid(data []float64) [][]float64 {

	isContinuous := math.Floor(float64(w.NLon)*w.ΔLon) >= 360

	nLon := w.NLon
	if isContinuous {
		nLon++
	}

	grid := make([][]float64, w.NLat)

	p := 0
	max := 0.0
	for j := uint32(0); j < w.NLat; j++ {
		grid[j] = make([]float64, nLon)
		for i := uint32(0); i < w.NLon; i++ {
			grid[j][i] = data[p]
			if data[p] > max {
				max = data[p]
			}
			p++
		}
		if isContinuous {
			grid[j][w.NLon] = grid[j][0]
		}
	}
	//    fmt.Println("max", max, "lat", w.NLat, len(grid), "lon", w.NLon, len(grid[0]))
	return grid
}

func Init(date time.Time, file string) (Wind, error) {
	w := Wind{Date: date, File: file}
	gribfile, _ := os.Open("grib-data/" + file)
	messages, err := griblib.ReadMessages(gribfile)
	if err != nil {
		return w, err
	}
	for _, message := range messages {
		if message.Section0.Discipline == uint8(0) && message.Section4.ProductDefinitionTemplate.ParameterCategory == uint8(2) && message.Section4.ProductDefinitionTemplate.FirstSurface.Type == 103 && message.Section4.ProductDefinitionTemplate.FirstSurface.Value == 10 {
			grid0, _ := message.Section3.Definition.(*griblib.Grid0)
			w.Lat0 = float64(grid0.La1 / 1e6)
			w.Lon0 = float64(grid0.Lo1 / 1e6)
			w.ΔLat = float64(grid0.Di / 1e6)
			w.ΔLon = float64(grid0.Dj / 1e6)
			w.NLat = grid0.Nj
			w.NLon = grid0.Ni
			if message.Section4.ProductDefinitionTemplate.ParameterNumber == 2 {
				w.U = w.buildGrid(message.Section7.Data)
			} else if message.Section4.ProductDefinitionTemplate.ParameterNumber == 3 {
				w.V = w.buildGrid(message.Section7.Data)
			}
		}
	}
	return w, nil
}

func floorMod(a float64, n float64) float64 {
	return a - n*math.Floor(a/n)
}

func bilinearInterpolate(x float64, y float64, g00 []float64, g10 []float64, g01 []float64, g11 []float64) (float64, float64) {

	rx := (1 - x)
	ry := (1 - y)

	a := rx * ry
	b := x * ry
	c := rx * y
	d := x * y

	u := g00[0]*a + g10[0]*b + g01[0]*c + g11[0]*d
	v := g00[1]*a + g10[1]*b + g01[1]*c + g11[1]*d

	return u, v
}

func vectorToDegrees(u float64, v float64, d float64) float64 {

	velocityDir := math.Atan2(u/d, v/d)
	velocityDirToDegrees := velocityDir*180/math.Pi + 180
	return velocityDirToDegrees
}

func (w Wind) interpolate(lat float64, lon float64) (float64, float64) {

	i := math.Abs((lat - w.Lat0) / w.ΔLat)
	j := floorMod(lon-w.Lon0, 360.0) / w.ΔLon

	fi := uint32(i)
	fj := uint32(j)

	u00 := w.U[fi][fj]
	v00 := w.V[fi][fj]

	u01 := w.U[fi+1][fj]
	v01 := w.V[fi+1][fj]

	u10 := w.U[fi][fj+1]
	v10 := w.V[fi][fj+1]

	u11 := w.U[fi+1][fj+1]
	v11 := w.V[fi+1][fj+1]

	u, v := bilinearInterpolate(j-float64(fj), i-float64(fi), []float64{u00, v00}, []float64{u10, v10}, []float64{u01, v01}, []float64{u11, v11})

	return u, v
}

func midInterpolate(ws []*Wind, lat float64, lon float64, h float64) (float64, float64) {

	if len(ws) == 1 {
		return ws[0].interpolate(lat, lon)
	}

	u1, v1 := ws[0].interpolate(lat, lon)
	u2, v2 := ws[1].interpolate(lat, lon)
	u := u2*h + u1*(1-h)
	v := v2*h + v1*(1-h)

	return u, v
}

func Interpolate(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64, vent int) (float64, float64) {
	if vent == 1 {
		return Interpolate1(w1, w2, lat, lon, h)
	}
	if vent == 2 {
		return Interpolate2(w1, w2, lat, lon, h)
	}
	if vent == 3 {
		return Interpolate3(w1, w2, lat, lon, h)
	}
	if vent == 4 {
		return Interpolate4(w1, w2, lat, lon, h)
	}
	return Interpolate0(w1, w2, lat, lon, h)
}

func Interpolate0(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64) (float64, float64) {

	u, v := midInterpolate(w1, lat, lon, h)

	if w2 != nil {
		u2, v2 := midInterpolate(w2, lat, lon, h)
		u = u2*h + u*(1-h)
		v = v2*h + v*(1-h)
	}
	d := math.Sqrt(u*u + v*v)

	return vectorToDegrees(u, v, d), d

}

func Interpolate1(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64) (float64, float64) {

	// cas j'utilise toujours anciennes polaires

	u, v := midInterpolate(w1[len(w1)-1:len(w1)], lat, lon, 1-h)

	if w2 != nil {
		u2, v2 := midInterpolate(w2, lat, lon, h)
		u = u2*h + u*(1-h)
		v = v2*h + v*(1-h)
	}
	d := math.Sqrt(u*u + v*v)

	return vectorToDegrees(u, v, d), d
}

func Interpolate2(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64) (float64, float64) {

	// cas j'utilise toujours nouvelles polaires

	u, v := midInterpolate(w1, lat, lon, 1-h)

	if w2 != nil {
		u2, v2 := midInterpolate(w2[0:1], lat, lon, h)
		u = u2*h + u*(1-h)
		v = v2*h + v*(1-h)
	}
	d := math.Sqrt(u*u + v*v)

	return vectorToDegrees(u, v, d), d
}

func Interpolate3(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64) (float64, float64) {


	u, v := midInterpolate(w1[len(w1)-1:len(w1)], lat, lon, 1-h)

	if w2 != nil {
		u2, v2 := midInterpolate(w2[len(w2)-1:len(w2)], lat, lon, h)
		u = u2*h + u*(1-h)
		v = v2*h + v*(1-h)
	}
	d := math.Sqrt(u*u + v*v)

	return vectorToDegrees(u, v, d), d
}

func Interpolate4(w1 []*Wind, w2 []*Wind, lat float64, lon float64, h float64) (float64, float64) {

	// cas j'utilise toujours nouvelles polaires : -1 ... 0 .x. 1

	u, v := midInterpolate(w1, lat, lon, 1-h)

	if w2 != nil {
		u2, v2 := midInterpolate(w2[len(w2)-1:len(w2)], lat, lon, h)
		u = u2*h + u*(1-h)
		v = v2*h + v*(1-h)
	}
	d := math.Sqrt(u*u + v*v)

	return vectorToDegrees(u, v, d), d
}
