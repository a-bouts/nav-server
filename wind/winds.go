package wind

import (
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/jasonlvhit/gocron"
	log "github.com/sirupsen/logrus"
)

type ForecastWinds []*Wind

func (w ForecastWinds) String() string {
	res := ""
	res += w[0].Date.Format("2006010215") + "(" + w[0].File
	if len(w) > 1 {
		res += "," + w[1].File
	}
	res += ")"
	return res
}

type Winds struct {
	winds map[string](ForecastWinds)
	lock  sync.RWMutex
}

func InitWinds() *Winds {
	w := &Winds{
		winds: LoadAll(),
		lock:  sync.RWMutex{},
	}

	s := gocron.NewScheduler()
	jobxx := s.Every(15).Seconds()
	jobxx.Do(w.Merge)

	go s.Start()

	return w
}

func (w *Winds) FindWinds(m time.Time) (ForecastWinds, ForecastWinds, float64) {
	w.lock.Lock()
	defer w.lock.Unlock()

	stamp := m.Format("2006010215")

	keys := make([]string, 0, len(w.winds))
	for k := range w.winds {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	if keys[0] > stamp {
		return w.winds[keys[0]], nil, 0
	}
	for i := range keys {
		if keys[i] > stamp {
			h := m.Sub(w.winds[keys[i-1]][0].Date).Minutes()
			delta := w.winds[keys[i]][0].Date.Sub(w.winds[keys[i-1]][0].Date).Minutes()
			return w.winds[keys[i-1]], w.winds[keys[i]], h / delta
		}
	}
	return w.winds[keys[len(keys)-1]], nil, 0
}

func (w *Winds) Merge() error {
	w.lock.Lock()
	defer w.lock.Unlock()

	// On supprime les fichiers qui ne sont plus là
	var toRemove []string
	for k, ws := range w.winds {
		if _, err := os.Stat("grib-data/" + ws[0].File); os.IsNotExist(err) {
			toRemove = append(toRemove, k)
		}
	}
	for _, k := range toRemove {
		log.Println("Remove from winds", k)
		delete(w.winds, k)
	}

	var files []string
	err := filepath.Walk("grib-data/", func(path string, info os.FileInfo, err error) error {
		if err != nil {
			log.WithError(err).Errorf("Error walking file '%s'", path)
		} else if info.Mode().IsRegular() && !strings.HasSuffix(info.Name(), ".tmp") {
			files = append(files, info.Name())
		}
		return nil
	})
	if err != nil {
		log.WithError(err).Error("Error walking grib files")
		return nil
	}

	sort.Strings(files)

	forecasts := make(map[int][]string)

	for cpt, f := range files {

		d := strings.Split(f, ".")[0]

		h, err := strconv.Atoi(strings.Split(f, ".")[1][1:])
		if err != nil {
			log.WithError(err).Errorf("Error getting hour from file '%s'", f)
			return nil
		}
		t, err := time.Parse("2006010215", d)
		if err != nil {
			log.WithError(err).Errorf("Error parsing date '%s'", d)
			return nil
		}

		t = t.Add(time.Hour * time.Duration(h))

		forecastHour := int(math.Round(t.Sub(time.Now()).Hours()))

		if forecastHour < -3 && cpt < len(files)-1 {
			continue
		}

		_, found := forecasts[forecastHour]

		//quand c'est la prévision précédente, on la conserve meme si une nouvelle prévision est arrivé
		if !found || forecastHour >= 0 {
			forecasts[forecastHour] = append(forecasts[forecastHour], f)
		}
	}

	var keys []int
	for k := range forecasts {
		keys = append(keys, k)
	}
	sort.Ints(keys)
	for _, k := range keys {
		for _, file := range forecasts[k] {
			d := strings.Split(file, ".")[0]
			date, _ := time.Parse("2006010215", d)
			f, _ := strconv.Atoi(strings.Split(file, ".")[1][1:])
			date = date.Add(time.Hour * time.Duration(f))
			sdate := date.Format("2006010215")

			ws, found := w.winds[sdate]
			if found {
				if len(ws) == 2 || ws[0].File == file {
					continue
				}
			}

			wind, err := Init(date, file)
			if err != nil {
				log.WithError(err).Errorf("Error loading grib file '%s'", file)
			} else {
				log.Debugf("Init %s %s", sdate, wind.File)
				w.winds[sdate] = append(w.winds[sdate], &wind)
			}
		}
	}

	return nil
}

func LoadAll() map[string](ForecastWinds) {
	winds := make(map[string](ForecastWinds))
	var files []string
	err := filepath.Walk("grib-data/", func(path string, info os.FileInfo, err error) error {
		if err != nil {
			log.WithError(err).Errorf("Error walking file '%s'", path)
		} else if info.Mode().IsRegular() && !strings.HasSuffix(info.Name(), ".tmp") {
			files = append(files, info.Name())
		}
		return nil
	})
	if err != nil {
		log.WithError(err).Error("Error walking grib files")
		return nil
	}

	sort.Strings(files)

	forecasts := make(map[int][]string)

	for cpt, f := range files {

		d := strings.Split(f, ".")[0]

		log.Debug(f)

		h, err := strconv.Atoi(strings.Split(f, ".")[1][1:])
		if err != nil {
			log.WithError(err).Errorf("Error getting hour from file '%s'", f)
			return nil
		}
		t, err := time.Parse("2006010215", d)
		if err != nil {
			log.WithError(err).Errorf("Error parsing date '%s'", d)
			return nil
		}

		t = t.Add(time.Hour * time.Duration(h))

		forecastHour := int(math.Round(t.Sub(time.Now()).Hours()))

		if forecastHour < -3 && cpt < len(files)-1 {
			continue
		}

		_, found := forecasts[forecastHour]

		//quand c'est la prévision courante, on la conserve meme si une nouvelle prévision est arrivé
		if !found || forecastHour >= 0 {
			forecasts[forecastHour] = append(forecasts[forecastHour], f)
		}
	}

	var keys []int
	for k := range forecasts {
		keys = append(keys, k)
	}
	sort.Ints(keys)
	for _, k := range keys {
		for _, file := range forecasts[k] {
			d := strings.Split(file, ".")[0]
			date, _ := time.Parse("2006010215", d)
			f, _ := strconv.Atoi(strings.Split(file, ".")[1][1:])
			date = date.Add(time.Hour * time.Duration(f))
			sdate := date.Format("2006010215")
			wind, err := Init(date, file)
			if err != nil {
				log.WithError(err).Errorf("Error loading grib file '%s'", file)
			} else {
				log.Debugf("Init %s %s", sdate, wind.File)
				winds[sdate] = append(winds[sdate], &wind)
			}
		}
	}
	return winds
}
