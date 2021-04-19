package main
 
import (
	"fmt"
	"os"
	"image"
	"image/color"
	"image/png"
	"math/rand"
	"math"
	"sync"
	"time"
   	)

type Camera struct {
	Orig, Dir, Offsetx, Offsety Vector3d
}
func RayOn(c Camera, u, v float64) Ray {
	return Ray{c.Orig, Norm(Add(c.Dir, Add(Mult(c.Offsetx, u), Mult(c.Offsety, v))))}
}

type Material interface {	
	ResolveHit(hr *HitRecord, r *Ray) (bool, Vector3d)
}
type Hittable interface {
	Norm(point Vector3d) Vector3d
	RayIntersect(r Ray, tmin, tmax float64) (bool, HitRecord)
}

type HitRecord struct {
	Point, Norm Vector3d
	T float64
	Mat Material
}
type Sphere struct {
	cent Vector3d
	rad float64
	mat Material
}
func (sph Sphere) Norm(point Vector3d) Vector3d {
	return Mult(Sub(point, sph.cent), 1.0 / sph.rad)
}
func (sph Sphere) RayIntersect(r Ray, tmin, tmax float64) (bool, HitRecord) {
	transOrig := Sub(r.Orig, sph.cent)
	a := SquaredLength(r.Dir)
	b := 2.0 * Dot(transOrig, r.Dir)
	c := SquaredLength(transOrig) - sph.rad * sph.rad
	D := (b * b - 4.0 * a * c)
	if D >= 0 {
		D = math.Sqrt(D)
		tans := (-b - D) / (2.0 * a)
		if tmin <= tans && tans <= tmax {
			pOnSphere := PointOn(r, tans)
			return true, HitRecord{pOnSphere, sph.Norm(pOnSphere), tans, sph.mat}
		}
		tans  = (-b + D) / (2.0 * a)
		if tmin <= tans && tans <= tmax {
			pOnSphere := PointOn(r, tans)
			return true, HitRecord{pOnSphere, sph.Norm(pOnSphere), tans, sph.mat}
		}
	}
	return false, HitRecord{Vector3d{0, 0, 0}, Vector3d{0, 0, 0}, 0, nil}
}

type Lambertian struct {
	Col Vector3d
	P float64
}
func (lamb Lambertian) ResolveHit(hr *HitRecord, r *Ray) (bool, Vector3d) {
	r.Orig = hr.Point
	r.Dir = Norm(Add(hr.Norm, Mult(RandomInUnit(), 1)))
	return true, Mult(lamb.Col, lamb.P)
}

type Metal struct {
	Col Vector3d
	Fuzz float64
}
func (met Metal) ResolveHit(hr *HitRecord, r *Ray) (bool, Vector3d) {
	reflected := Reflect(Norm(r.Dir), hr.Norm)
	reflected = Add(reflected, Mult(RandomInUnit(), met.Fuzz))
	r.Orig = hr.Point
	r.Dir =  reflected
	return (Dot(r.Dir, hr.Norm) > 0), met.Col
}

func Schlick(cos, n1, n2 float64) float64 {
	r0 := (n2 - n1) / (n2 + n1)
	r0 = (r0 * r0)
	return r0 + (1.0 - r0) * math.Pow(1.0 - cos, 5)
}
type Dielectric struct {
	Col Vector3d
	N1, N2 float64 ///n1 - outside, n2 - inside
}
func (diel Dielectric) ResolveHit(hr *HitRecord, r *Ray) (bool, Vector3d) {
	dot := Dot(r.Dir, hr.Norm)
	if dot > 0 {
		realNorm := Sub(Vector3d{0, 0, 0}, hr.Norm)
		ratio := diel.N2 / diel.N1
		cos := diel.N2 * dot / Length(r.Dir)
		refracted, refrDir := Refract(r.Dir, realNorm, ratio)
		
		reflectProb := 1.0
		if refracted { reflectProb = Schlick(cos, diel.N1, diel.N2)}

		if rand.Float64() < reflectProb {
			r.Dir = Norm(Reflect(r.Dir, hr.Norm))
		} else {
			r.Dir = Norm(refrDir)
		}
	} else {
		realNorm := hr.Norm
		ratio := diel.N1 / diel.N2				
		cos := -diel.N1 * dot / Length(r.Dir)
		refracted, refrDir := Refract(r.Dir, realNorm, ratio)
		
		reflectProb := 1.0
		if refracted { reflectProb = Schlick(cos, diel.N1, diel.N2)}
		
		if rand.Float64() < reflectProb {
			r.Dir = Norm(Reflect(r.Dir, hr.Norm))
		} else {
			r.Dir = Norm(refrDir)
		}
	}
	r.Orig = hr.Point
	return true, diel.Col
}

const (
		width = 800
		height = 600
		aaSamples = 10
		recDepth = 50
		recSamples = 3
		threadsCount = 8
	)

var (
		camera = Camera{Vector3d{0, 0, 1}, Vector3d{0, 0, -2}, Vector3d{2, 0, 0}, 
		                Mult(Norm(Vector3d{0, 1, 0}), Length(Vector3d{2, 0, 0}) * float64(height) / float64(width))}
		hittables []Hittable
	)

func RayShoot(r Ray, depth int16) Vector3d {
	if depth == 0 {
		return Vector3d{0, 0, 0}
	}
	hr := HitRecord{Vector3d{0, 0, 0}, Vector3d{0, 0, 0}, 1e9 + 10, nil}
	for _, nowHittable := range hittables {	
		nowHit, hitRec := nowHittable.RayIntersect(r, 0.01, hr.T)
		if nowHit {
			hr = hitRec
		}
	}
	if(hr.T < 1e9) {		
		
		nowRecSamples := 1 + (recSamples >> uint16((recDepth - depth) / 2))
		colNow := Vector3d{0, 0, 0}
		cpyr := r
		for sample := 0; sample < nowRecSamples; sample ++ {
			r = cpyr
			refl, col := hr.Mat.ResolveHit(&hr, &r)
			if refl {
				rayCol := MultVec(RayShoot(r, depth - 1), col)
				colNow.X += rayCol.X; colNow.Y += rayCol.Y; colNow.Z += rayCol.Z 
			}
		}
		colNow = Mult(colNow, 1.0 / float64(nowRecSamples))
		return colNow
	} else {
		t := (Norm(r.Dir).Y + 1.0) / 2.0
		return Add(Mult(Vector3d{1.0, 1.0, 1.0}, (1 - t)), Mult(Vector3d{0.5, 0.5, 0.8}, t))
		return Vector3d{1, 1, 1}
	}
}

type SafeImage struct {
	mu sync.Mutex
	wg sync.WaitGroup
	img *image.RGBA
	cnt int
}

func renderPartImage(simg *SafeImage, startCol, stopCol int) {
	for x := startCol; x < stopCol; x ++ {		
		for y := 0; y < height; y ++ {
			colNow := Vector3d{0, 0, 0}
			for sample := 0; sample < aaSamples; sample ++ {
				transx := ( (float64(x) + rand.Float64() - float64( width) / 2.0)) / (float64( width) / 2.0)
				transy := (-(float64(y) + rand.Float64() - float64(height) / 2.0)) / (float64(height) / 2.0)

				rayCol := RayShoot(RayOn(camera, transx, transy), recDepth)
				colNow.X += rayCol.X; colNow.Y += rayCol.Y; colNow.Z += rayCol.Z
			}
			FixCol(&colNow, aaSamples)
			//Probably not needed to mutex lock
			//simg.mu.Lock()
			simg.img.Set(int(x), int(y), 
			             color.RGBA{uint8(255.0 * colNow.X), uint8(255.0 * colNow.Y), uint8(255.0 * colNow.Z), 255})
			simg.cnt ++
			//simg.mu.Unlock()
		}
	}
	simg.wg.Done()
}

func Spinner(simg *SafeImage, delay time.Duration) {
    for {
        for _, r := range `-\|/` {
			simg.mu.Lock()
			cnt := simg.cnt
			simg.mu.Unlock()
			if cnt == width * height {
				break
			}
            fmt.Printf("\r%c %d from %d", r, cnt, width * height)
            time.Sleep(delay)
        }
    }
    fmt.Printf("\r \r")
}

func renderImage() {///TODO Manage better goroutines
	simg := SafeImage{img : image.NewRGBA(image.Rectangle{image.Point{0, 0}, image.Point{width, height}})}
	for thr := 0; thr < threadsCount; thr ++ {
		simg.wg.Add(1)
		if thr != threadsCount - 1 {
			go renderPartImage(&simg, width / threadsCount * thr, width / threadsCount * (thr + 1))
		} else {
			go renderPartImage(&simg, width / threadsCount * thr, width)
		}
	}
	go Spinner(&simg, time.Second / 2)
	simg.wg.Wait()
	final, _ := os.Create("image.png")
	png.Encode(final, simg.img)
}

func initScene() {
	diff  := Lambertian{Vector3d{0.1, 0.2, 0.5}, 0.3}
	diff1 := Lambertian{Vector3d{0.8, 0.8, 0.0}, 0.3}
	diff2 := Metal{Vector3d{0.8, 0.6, 0.2}, 0.3}
	diff3 := Dielectric{Vector3d{1, 1, 1}, 1, 1.5}
	diff4 := Dielectric{Vector3d{1, 1, 1}, 1.5, 1}
	hittables = append(hittables, Sphere{Vector3d{-1, 0, -1.5}, 0.5, diff3}, Sphere{Vector3d{-1, 0, -1.5}, 0.45, diff4})
	hittables = append(hittables, Sphere{Vector3d{0, 0, -1.5}, 0.5, diff}, Sphere{Vector3d{1, 0, -1.5}, 0.5, diff2}) 
	hittables = append(hittables, Sphere{Vector3d{0, -100.5, -2}, 100, diff1})
	hittables = append(hittables, Sphere{Vector3d{-0.3, -0.4, -0.5}, 0.1, diff}, Sphere{Vector3d{0.4, -0.4, -0.5}, 0.1, diff2}) 
	hittables = append(hittables, Sphere{Vector3d{0, -0.4, -0.4}, 0.1, diff1}) 
}

func main() {
	initScene()
	fmt.Println("Started")
	start := time.Now()
	renderImage()
	elapsed := time.Since(start)
	fmt.Println("Ended", elapsed)
}