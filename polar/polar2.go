package polar

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"math"
)

type Boat2 struct {
	Label                   string    `json:"label"`
	GlobalSpeedRatio        float64   `json:"globalSpeedRatio"`
	IceSpeedRatio           float64   `json:"iceSpeedRatio"`
	AutoSailChangeTolerance float64   `json:"autoSailChangeTolerance"`
	BadSailTolerance        float64   `json:"badSailTolerance"`
	MaxSpeed                float64   `json:"maxSpeed"`
	Foil                    Foil      `json:"foil"`
	Hull                    Hull      `json:"hull"`
	Winch                   Winch     `json:"winch"`
	Tws                     []float64 `json:"tws"`
	Twa                     []float64 `json:"twa"`
	Sail                    []Sail    `json:"sail"`
}

type Foil struct {
	SpeedRatio float64 `json:"speedRatio"`
	TwaMin     float64 `json:"twaMin"`
	TwaMax     float64 `json:"twaMax"`
	TwaMerge   float64 `json:"twaMerge"`
	TwsMin     float64 `json:"twsMin"`
	TwsMax     float64 `json:"twsMax"`
	TwsMerge   float64 `json:"twsMerge"`
}

type Hull struct {
	SpeedRatio float64 `json:"speedRatio"`
}

type Winch struct {
	Tack       Penalty `json:"tack"`
	Gybe       Penalty `json:"gybe"`
	SailChange Penalty `json:"sailChange"`
}

type Penalty struct {
	StdTimerSec int     `json:"stdTimerSec"`
	StdRatio    float64 `json:"stdRatio"`
	ProTimerSec int     `json:"proTimerSec"`
	ProRatio    float64 `json:"proRatio"`
}

type Sail struct {
	Id    int         `json:"id"`
	Name  string      `json:"name"`
	Speed [][]float64 `json:"speed"`
}

func Load(o Options) Boat2 {

	fmt.Println("Load " + o.Race + ".json")

	data, err := ioutil.ReadFile("polars/" + o.Race + ".json")
	if err != nil {
		fmt.Print(err)
	}

	var boat Boat2

	err = json.Unmarshal(data, &boat)
	if err != nil {
		fmt.Println("error:", err)
	}
	return boat
}

func interpolationIndex(values []float64, value float64) (int, int, float64) {

	i := 0
	for values[i] < value {
		i++
		if i == len(values) {
			if values[i-1] < value {
				return i - 1, 0, 1
			}
			return i - 1, i, (values[i] - value) / (values[i] - values[i-1])
		}
	}

	if i > 0 {
		return i - 1, i, (values[i] - value) / (values[i] - values[i-1])
	}

	return 0, 0, 0
}

func foil2(boat Boat2, twa float64, ws float64) float64 {
	ct := 0.0
	cv := 0.0
	if twa <= boat.Foil.TwaMin-boat.Foil.TwaMerge {
		return 1.0
	} else if twa < boat.Foil.TwaMin {
		ct = float64(twa-(boat.Foil.TwaMin-boat.Foil.TwaMerge)) / boat.Foil.TwaMerge
	} else if twa < boat.Foil.TwaMax {
		ct = 1
	} else if twa < boat.Foil.TwaMax+boat.Foil.TwaMerge {
		ct = float64(boat.Foil.TwaMax+boat.Foil.TwaMerge-twa) / boat.Foil.TwaMerge
	} else {
		return 1.0
	}
	if ws <= boat.Foil.TwsMin-boat.Foil.TwsMerge {
		return 1.0
	} else if ws < boat.Foil.TwsMin {
		cv = (ws - (boat.Foil.TwsMin - boat.Foil.TwsMerge)) / boat.Foil.TwsMerge
	} else if ws < boat.Foil.TwsMax {
		cv = 1
	} else if ws < boat.Foil.TwsMax+boat.Foil.TwsMerge {
		cv = (boat.Foil.TwsMax + boat.Foil.TwsMerge - ws) / boat.Foil.TwsMerge
	} else {
		return 1.0
	}
	return 1.0 + (boat.Foil.SpeedRatio-1)*ct*cv
}

func (boat Boat2) GetBoatSpeed(twa float64, ws float64, context Boat, isInIceLimits bool) (float64, byte, uint8) {
	// convert m/s to kts
	ws = ws * 1.9438444924406

	t := twa
	if t < 0 {
		t = -1 * t
	}
	if t > 180 {
		t = 360 - t
	}

	twsIndex0, twsIndex1, twsFactor := interpolationIndex(boat.Tws, ws)
	twaIndex0, twaIndex1, twaFactor := interpolationIndex(boat.Twa, t)

	maxBs := 0.0
	maxS := byte(0)

	for s, sail := range boat.Sail {
		if (sail.Name == "LightJib" || sail.Name == "LightGnk") && (context.Sails&1) != 1 {
			continue
		}
		if (sail.Name == "Staysail" || sail.Name == "HeavyGnk") && (context.Sails&4) != 4 {
			continue
		}
		if sail.Name == "Code0" && (context.Sails&2) != 2 {
			continue
		}

		bs := (sail.Speed[twaIndex0][twsIndex0]*twsFactor+sail.Speed[twaIndex0][twsIndex1]*(1-twsFactor))*twaFactor + (sail.Speed[twaIndex1][twsIndex0]*twsFactor+sail.Speed[twaIndex1][twsIndex1]*(1-twsFactor))*(1-twaFactor)

		if bs > maxBs {
			maxBs = bs
			maxS = byte(s)
		}
	}

	maxBs *= boat.GlobalSpeedRatio
	if isInIceLimits {
		maxBs *= boat.IceSpeedRatio
	}
	if context.Hull {
		maxBs *= boat.Hull.SpeedRatio
	}
	f := foil2(boat, t, ws)
	if context.Foil {
		maxBs *= f
	}

	// if math.Round(twa) == 60 {
	// 	fmt.Printf("twa %d, ws %f, bs %f\n", int(math.Round(twa)), ws, maxBs)
	// 	log.Fatal()
	// }
	//

	return maxBs, maxS, uint8(math.Round((f - 1.0) * 100 / (boat.Foil.SpeedRatio - 1)))
}
