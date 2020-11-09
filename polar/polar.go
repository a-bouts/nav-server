package polar

import (
	"fmt"
	"io/ioutil"
	"math"
)

type Polar interface {
	GetBoatSpeed(twa float64, ws float64, boat Boat, isInIceLimits bool) (float64, byte, uint8)
}

type Zezo struct {
	options    Options
	polar_data [8][]byte
}

type Options struct {
	Race string
	Sail int
}

type Boat struct {
	Foil  bool
	Hull  bool
	Sails int
}

func Init(o Options) Zezo {
	var p [8][]byte
	i := 0
	for i < 8 {

		b, err := ioutil.ReadFile("polars/" + o.Race + "_" + fmt.Sprintf("%d", i))
		if err != nil {
			fmt.Print(err)
		}
		p[i] = b
		i++
	}
	z := Zezo{}
	z.options = o
	z.polar_data = p
	return z
}

func (z Zezo) GetBoatSpeed(twa float64, ws float64, boat Boat, isInIceLimits bool) (float64, byte, uint8) {
	return z.GetBoatSpeed2(twa, ws*3.6, boat, z.options.Sail, isInIceLimits)
}

func (z Zezo) GetOptimBoatSpeed(twa float64, ws float64, boat Boat, s byte, winchMalus float64) (float64, byte, uint8) {
	o := 0
	if s == 2 || s == 3 {
		o = 1
	}
	if s == 4 {
		o = 2
	}
	if s == 5 || s == 6 {
		o = 4
	}
	bs1, s1, f1 := z.GetBoatSpeed2(twa, ws, boat, o, false)
	bs2, s2, f2 := z.GetBoatSpeed(twa, ws, boat, false)

	d1 := bs1 * 1.852 * 1.0 * 1000.0
	d2 := bs2 * 1.852 * 1.0 * 1000.0
	if s1 != s {
		d1 = bs1 * 1.852 * (1.0*60.0 - winchMalus/2) / 60 * 1000.0
	}
	if s2 != s {
		d2 = bs2 * 1.852 * (1.0*60.0 - winchMalus/2) / 60 * 1000.0
	}

	if d1 > d2 {
		return bs1, s1, f1
	} else {
		return bs2, s2, f2
	}
}

func (z Zezo) GetBoatSpeed2(twa float64, ws float64, boat Boat, s int, isInIceLimits bool) (float64, byte, uint8) {
	p := z.polar_data[s]
	t := twa
	if t < 0 {
		t = -1 * t
	}
	if t > 180 {
		t = 360 - t
	}

	if ws >= 129 {
		ws = 129
	}

	var offset int

	ws1 := int(math.Floor(ws))
	frac := ws - float64(ws1)
	offset = 4 * (130*int(math.Round(t)) + ws1)
	bs := (float64(p[offset+1])*256 + float64(p[offset+2])) / 100
	bs1 := (float64(p[offset+5])*256 + float64(p[offset+6])) / 100
	sail := p[offset]
	retbs := bs + frac*(bs1-bs)
	if isInIceLimits {
		retbs *= 0.3
	}
	f := foil(t, ws/1.852)
	if boat.Foil {
		retbs *= f
	}
	if boat.Hull {
		retbs *= 1.003
	}
	return retbs, sail, uint8(math.Round((f - 1.0) * 100 / 0.04))
}

func foil(twa, ws float64) float64 {
	ct := 0.0
	cv := 0.0
	if twa <= 70 {
		return 1.0
	} else if twa < 80 {
		ct = float64(twa-70) / 10.0
	} else if twa < 160 {
		ct = 1
	} else if twa < 170 {
		ct = float64(170-twa) / 10.0
	} else {
		return 1.0
	}
	if ws <= 11 {
		return 1.0
	} else if ws < 16 {
		cv = (ws - 11) / 5
	} else if ws < 35 {
		cv = 1
	} else if ws < 40 {
		cv = (40 - ws) / 5
	} else {
		return 1.0
	}
	return 1.0 + 0.04*ct*cv
}
