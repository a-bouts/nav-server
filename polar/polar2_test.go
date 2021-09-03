package polar

import (
	"math"
	"testing"
)

func TestInterpolationIndex(t *testing.T) {

	array := []float64{0, 4, 8}

	i0, i1, d := interpolationIndex(array, 0)
	if i0 != 0 || i1 != 0 || d != 0.0 {
		t.Errorf("interpolationIndex(0) = (%d, %d, %f); want (0, 0, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 1)
	if i0 != 0 || i1 != 1 || d != 0.75 {
		t.Errorf("interpolationIndex(1) = (%d, %d, %f); want (0, 1, 0.75)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 2)
	if i0 != 0 || i1 != 1 || d != 0.5 {
		t.Errorf("interpolationIndex(2) = (%d, %d, %f); want (0, 1, 0.5)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 3)
	if i0 != 0 || i1 != 1 || d != 0.25 {
		t.Errorf("interpolationIndex(3) = (%d, %d, %f); want (0, 1, 0.25)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 4)
	if i0 != 0 || i1 != 1 || d != 0.0 {
		t.Errorf("interpolationIndex(4) = (%d, %d, %f); want (0, 1, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 5)
	if i0 != 1 || i1 != 2 || d != 0.75 {
		t.Errorf("interpolationIndex(5) = (%d, %d, %f); want (1, 2, 0.75)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 8)
	if i0 != 1 || i1 != 2 || d != 0.0 {
		t.Errorf("interpolationIndex(8) = (%d, %d, %f); want (1, 2, 0.0)", i0, i1, d)
	}

	i0, i1, d = interpolationIndex(array, 9)
	if i0 != 2 || i1 != 0 || d != 1.0 {
		t.Errorf("interpolationIndex(9) = (%d, %d, %f); want (2, 0, 1.0)", i0, i1, d)
	}
}

func TestGetBoatSpeed(t *testing.T) {
	z := Load(Options{Race: "arctic", Sail: 0})

	ws, s, _ := z.GetBoatSpeed(0, 0, Boat{}, false)
	if ws != 0 || s != 0 {
		t.Errorf("GetBoatSpeed(0, 0) = (%f, %d); want (0, 0)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{}, false)
	if math.Round(ws*1000000) != 6220000 || s != 0 {
		t.Errorf("GetBoatSpeed(60, 39.1) = (%f, %d); want (14.600400, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{Hull: true}, false)
	if math.Round(ws*1000000) != 6238660 || s != 0 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.644201, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(60, 39.1, Boat{Foil: true}, false)
	if math.Round(ws*1000000) != 6220000 || s != 0 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.600400, 2)", ws, s)
	}

	ws, s, _ = z.GetBoatSpeed(43, 38.6, Boat{}, false)
	if math.Round(ws*1000000) != 4836000 || s != 0 {
		t.Errorf("GetBoatSpeed(60, 39.1, hull) = (%f, %d); want (14.600400, 2)", ws, s)
	}
}

func TestAddPenalty(t *testing.T) {
	z := Load(Options{Race: "arctic", Sail: 0})

	penalties := z.AddPenalty(nil, -45.0, 45.0, 0, 0, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Tack, Std) = (%d, %f); want (300, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -45.0, 45.0, 0, 0, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 75 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Tack, Pro) = (%d, %f); want (75, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 0, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Gybe, Std) = (%d, %f); want (300, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 0, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 75 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Gybe, Pro) = (%d, %f); want (75, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, 130.0, 130.0, 0, 1, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Sail, Std) = (%d, %f); want (300, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, 130.0, 130.0, 0, 1, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 75 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Sail, Pro) = (%d, %f); want (75, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 1, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.25 {
		t.Errorf("AddPenalty(Sail, Std) = (%d, %f); want (300, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 1, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 75 || penalties[0].Ratio != 0.25 {
		t.Errorf("AddPenalty(Sail, Pro) = (%d, %f); want (75, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}

	z = Load(Options{Race: "class40-2021", Sail: 0})

	penalties = z.AddPenalty(nil, -45.0, 45.0, 0, 0, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 240 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Tack, Std) = (%d, %f); want (240, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -45.0, 45.0, 0, 0, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 240 || penalties[0].Ratio != 0.7 {
		t.Errorf("AddPenalty(Tack, Pro) = (%d, %f); want (240, 0.7)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 0, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Gybe, Std) = (%d, %f); want (300, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 0, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.7 {
		t.Errorf("AddPenalty(Gybe, Pro) = (%d, %f); want (300, 0.7)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, 130.0, 130.0, 0, 1, 30, Boat{})
	if len(penalties) != 1 || penalties[0].DurationSec != 360 || penalties[0].Ratio != 0.5 {
		t.Errorf("AddPenalty(Sail, Std) = (%d, %f); want (360, 0.5)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, 130.0, 130.0, 0, 1, 30, Boat{WinchPro: true})
	if len(penalties) != 1 || penalties[0].DurationSec != 360 || penalties[0].Ratio != 0.7 {
		t.Errorf("AddPenalty(Sail, Pro) = (%d, %f); want (360, 0.7)", penalties[0].DurationSec, penalties[0].Ratio)
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 1, 30, Boat{})
	if len(penalties) != 2 || penalties[0].DurationSec != 300 || penalties[0].Ratio != 0.25 || penalties[1].DurationSec != 60 || penalties[1].Ratio != 0.5 {
		t.Error(penalties)
		t.Errorf("AddPenalty(Sail, Std) want (300, 0.25 | 60, 0.5)")
	}
	penalties = z.AddPenalty(nil, -130.0, 130.0, 0, 1, 30, Boat{WinchPro: true})
	if len(penalties) != 2 || penalties[0].DurationSec != 300 || int(penalties[0].Ratio*100000) != 48999 /* 0.7*0.7 */ || penalties[1].DurationSec != 60 || penalties[1].Ratio != 0.7 {
		t.Error(penalties)
		t.Errorf("AddPenalty(Sail, Pro) want (300, 0.49 | 60, 0.7)")
	}

	penalties = z.AddPenalty([]Penalty{Penalty{DurationSec: 160, Ratio: 0.5}}, -45.0, 45.0, 0, 0, 30, Boat{})
	if len(penalties) != 2 || penalties[0].DurationSec != 160 || penalties[0].Ratio != 0.25 || penalties[1].DurationSec != 80 || penalties[1].Ratio != 0.5 {
		t.Error(penalties)
		t.Errorf("MergePenalty want (160, 0.25 | 80, 0.5)")
	}

	penalties = z.AddPenalty([]Penalty{Penalty{DurationSec: 300, Ratio: 0.5}}, -45.0, 45.0, 0, 0, 30, Boat{})
	if len(penalties) != 2 || penalties[0].DurationSec != 240 || penalties[0].Ratio != 0.25 || penalties[1].DurationSec != 60 || penalties[1].Ratio != 0.5 {
		t.Error(penalties)
		t.Errorf("MergePenalty want (240, 0.25 | 60, 0.5)")
	}

	penalties = z.AddPenalty([]Penalty{Penalty{DurationSec: 40, Ratio: 0.25}, Penalty{DurationSec: 40, Ratio: 0.5}}, -45.0, 45.0, 0, 0, 30, Boat{})
	if len(penalties) != 3 || penalties[0].DurationSec != 40 || penalties[0].Ratio != 0.125 || penalties[1].DurationSec != 40 || penalties[1].Ratio != 0.25 || penalties[2].DurationSec != 160 || penalties[2].Ratio != 0.5 {
		t.Error(penalties)
		t.Errorf("MergePenalty want (40, 0.125 | 40, 0.25 | 160, 0.5)")
	}

}
