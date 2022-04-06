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
	Tack       Move    `json:"tack"`
	Gybe       Move    `json:"gybe"`
	SailChange Move    `json:"sailChange"`
	Lws        float64 `json:"lws"`
	Hws        float64 `json:"hws"`
}

type Move struct {
	StdTimerSec int        `json:"stdTimerSec"`
	StdRatio    float64    `json:"stdRatio"`
	ProTimerSec int        `json:"proTimerSec"`
	ProRatio    float64    `json:"proRatio"`
	Std         *Penalties `json:"std"`
	Pro         *Penalties `json:"pro"`
}

type Penalties struct {
	Lw PenaltyValue `json:"lw"`
	Hw PenaltyValue `json:"hw"`
}

type PenaltyValue struct {
	Ratio float64 `json:"ratio"`
	Timer int     `json:"timer"`
}

type Sail struct {
	Id     int         `json:"id"`
	Name   string      `json:"name"`
	Speed  [][]float64 `json:"speed"`
	option byte
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
	for s, sail := range boat.Sail {
		if sail.Name == "LIGHT_JIB" || sail.Name == "LIGHT_GNK" || sail.Name == "LightJib" || sail.Name == "LightGnk" {
			boat.Sail[s].option = 1
		} else if sail.Name == "STAYSAIL" || sail.Name == "HEAVY_GNK" || sail.Name == "Staysail" || sail.Name == "HeavyGnk" {
			boat.Sail[s].option = 4
		} else if sail.Name == "CODE_0" || sail.Name == "Code0" {
			boat.Sail[s].option = 2
		}
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

func (boat Boat2) GetBoatSpeed(twa float64, ws float64, context Boat, currentSail byte, isInIceLimits bool) (float64, byte, uint8, uint8) {
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
	unMoinsTwsFactor1 := 1 - twsFactor
	unMoinsTwaFactor1 := 1 - twaFactor

	var maxBs float64
	var currentBs float64
	var maxS byte

	l := len(boat.Sail)
	for s := 0; s < l; s++ {

		sail := &boat.Sail[s]
		if sail.option&context.Sails != sail.option {
			continue
		}

		ti0 := sail.Speed[twaIndex0]
		ti1 := sail.Speed[twaIndex1]
		bs := (ti0[twsIndex0]*twsFactor+ti0[twsIndex1]*(unMoinsTwsFactor1))*twaFactor + (ti1[twsIndex0]*twsFactor+ti1[twsIndex1]*(unMoinsTwsFactor1))*(unMoinsTwaFactor1)

		if byte(s) == currentSail {
			currentBs = bs
		}
		if bs > maxBs {
			maxBs = bs
			maxS = byte(s)
		}
	}

	boost := 1.0
	if context.AutoSail {
		boost = maxBs / currentBs
		if boost <= boat.AutoSailChangeTolerance {
			maxS = currentSail
		} else {
			boost = 1.0
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

	return maxBs, maxS, uint8(math.Round((f - 1.0) * 100 / (boat.Foil.SpeedRatio - 1))), uint8(math.Round((boost - 1.0) * 100 / (boat.AutoSailChangeTolerance - 1.0)))
}

func (boat Boat2) getPenaltyValues(p *Move, wsKt float64, context Boat) (int, float64) {

	h := 0.0

	if wsKt < boat.Winch.Lws {
		h = 0.0
	} else if wsKt > boat.Winch.Hws {
		h = 1.0
	} else {
		h = (wsKt - boat.Winch.Lws) / (boat.Winch.Hws - boat.Winch.Lws)
	}

	lwr := p.ProRatio
	hwr := p.ProRatio
	lwt := p.ProTimerSec
	hwt := p.ProTimerSec

	if !context.WinchPro {
		lwr = p.StdRatio
		hwr = p.StdRatio
		lwt = p.StdTimerSec
		hwt = p.StdTimerSec
	}

	if context.WinchPro && p.Pro != nil {
		lwr = p.Pro.Lw.Ratio
		hwr = p.Pro.Hw.Ratio
		lwt = p.Pro.Lw.Timer
		hwt = p.Pro.Hw.Timer
	} else if !context.WinchPro && p.Std != nil {
		lwr = p.Std.Lw.Ratio
		hwr = p.Std.Hw.Ratio
		lwt = p.Std.Lw.Timer
		hwt = p.Std.Hw.Timer
	}

	sinH := (1 - math.Sin(math.Pi/2+math.Pi*h)) / 2

	return int(sinH*float64(hwt) + (1-sinH)*float64(lwt)), sinH*hwr + (1-sinH)*lwr
}

type Penalty struct {
	DurationSec int
	Ratio       float64
	Type        byte
}

func MergePenalties(penalties *[]Penalty, index int, durationSec int, ratio float64, penaltyType byte) {

	if durationSec == 0 {
		return
	}

	if penalties == nil {
		*penalties = []Penalty{Penalty{DurationSec: durationSec, Ratio: ratio, Type: penaltyType}}
	} else if len(*penalties) == index {
		*penalties = append(*penalties, Penalty{DurationSec: durationSec, Ratio: ratio, Type: penaltyType})
	} else if (*penalties)[index].DurationSec <= durationSec {
		(*penalties)[index].Ratio *= ratio
		(*penalties)[index].Type |= penaltyType
		MergePenalties(penalties, index+1, durationSec-(*penalties)[index].DurationSec, ratio, penaltyType)
	} else {
		*penalties = append([]Penalty{Penalty{DurationSec: durationSec, Ratio: ratio * (*penalties)[index].Ratio, Type: penaltyType | (*penalties)[index].Type}}, (*penalties)...)
		(*penalties)[index+1].DurationSec -= durationSec
	}

}

func (boat Boat2) AddPenalty(penalties []Penalty, previousTwa float64, newTwa float64, previousSail byte, newSail byte, wsMs float64, context Boat) []Penalty {

	wsKt := wsMs * 1.9438444924406

	//fmt.Printf("AddPenalty (%f -> %f) (%d -> %d)\n", previousTwa, newTwa, previousSail, newSail)

	newPenalties := make([]Penalty, len(penalties))
	copy(newPenalties, penalties)

	if newTwa*previousTwa < 0 && math.Abs(newTwa) < 90 {
		d, r := boat.getPenaltyValues(&boat.Winch.Tack, wsKt, context)
		//fmt.Printf("Tack %d %f\n", d, r)

		MergePenalties(&newPenalties, 0, d, r, 1)

	} else if newTwa*previousTwa < 0 && math.Abs(newTwa) >= 90 {
		d, r := boat.getPenaltyValues(&boat.Winch.Gybe, wsKt, context)
		//fmt.Printf("Gybe %d %f\n", d, r)

		MergePenalties(&newPenalties, 0, d, r, 2)
	}

	if previousSail != newSail {
		d, r := boat.getPenaltyValues(&boat.Winch.SailChange, wsKt, context)
		//fmt.Printf("Sail %d %f\n", d, r)

		MergePenalties(&newPenalties, 0, d, r, 4)
	}

	return newPenalties
}
