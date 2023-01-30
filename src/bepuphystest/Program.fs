open System
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharp.Data.Adaptive

open BepuPhysics
open BepuUtilities
open System.Numerics

module Things = 
    let grav = Vector3(0.0f,0.0f,0.0001f)
    let mutable downwardVels = Vector3Wide()
    
    let trafos = 
        let rand = RandomSystem()
        let rr() = rand.UniformDouble()*3.0-1.5
        cval [|
            for x in -10.0 .. 2.0 .. 10.0 do
                for y in -10.0 .. 2.0 .. 10.0 do 
                    yield struct(Rot3d.Identity, V3d(x,y,5.0))
                    //yield Trafo3d.Translation(x,y,5.0)
            
            for x in -10.0 .. 2.0 .. 10.0 do
                for y in -10.0 .. 2.0 .. 10.0 do 
                    yield struct(Rot3d.Rotation(rand.UniformV3d().Normalized,rr()), V3d(x+rr(),y+rr(),10.0+rr()))
                    //yield Trafo3d.Translation(x+rr(),y+rr(),10.0+rr())
        |]


type MyCallbacks =
    struct
        interface IPoseIntegratorCallbacks with
            member this.AllowSubstepsForUnconstrainedBodies: bool = 
                false
            member this.AngularIntegrationMode: AngularIntegrationMode = 
                AngularIntegrationMode.Nonconserving
            member this.Initialize(simulation: Simulation): unit = 
                ()
            member this.IntegrateVelocity(bodyIndices: Numerics.Vector<int>, position: BepuUtilities.Vector3Wide, orientation: BepuUtilities.QuaternionWide, localInertia: BodyInertiaWide, integrationMask: Numerics.Vector<int>, workerIndex: int, dt: Numerics.Vector<float32>, velocity: byref<BodyVelocityWide>): unit = 
                //== set velocity ==
                velocity.Linear <- velocity.Linear + Things.downwardVels
                
                //== evacuate things ==
                let i = 0
                let rot = 
                    let axis = ref (Vector3Wide())
                    let angle = ref (Vector<float32>())
                    QuaternionWide.GetAxisAngleFromQuaternion(&orientation,axis,angle)
                    let ax = V3d(float (!axis).X.[i],float (!axis).Y.[i],float (!axis).Z.[i])
                    let an = float (!angle).[i]
                    Rot3d(an,ax)
                let pos = V3d(position.X.[i], position.Y.[i], position.Z.[i])
                transact (fun _ -> 
                    let narr = Things.trafos.Value |> Array.copy
                    narr.[bodyIndices.[i]] <- struct(rot,pos)
                    Things.trafos.Value <- narr
                )

            member this.IntegrateVelocityForKinematics: bool = 
                false
            member this.PrepareForIntegration(dt: float32): unit = 
                Things.downwardVels <- Vector3Wide.Broadcast(Things.grav*dt)
            
    end

type NarrowPhaseCallbacks =
    struct
        interface CollisionDetection.INarrowPhaseCallbacks with
            member this.AllowContactGeneration(workerIndex: int, a: Collidables.CollidableReference, b: Collidables.CollidableReference, speculativeMargin: byref<float32>): bool = 
                true
            member this.AllowContactGeneration(workerIndex: int, pair: CollisionDetection.CollidablePair, childIndexA: int, childIndexB: int): bool = 
                true
            member this.ConfigureContactManifold(workerIndex: int, pair: CollisionDetection.CollidablePair, manifold: byref<'TManifold>, pairMaterial: byref<CollisionDetection.PairMaterialProperties>): bool = 
                true
            member this.ConfigureContactManifold(workerIndex: int, pair: CollisionDetection.CollidablePair, childIndexA: int, childIndexB: int, manifold: byref<CollisionDetection.ConvexContactManifold>): bool = 
                true
            member this.Dispose(): unit = 
                ()
            member this.Initialize(simulation: Simulation): unit = 
                ()
    end

[<EntryPoint;STAThread>]
let main argv =
    Aardvark.Init()

    use app = new OpenGlApplication()
    use win = app.CreateGameWindow(4)

    let initialView = CameraView.lookAt (V3d(6,6,6)) V3d.Zero V3d.OOI
    let view = initialView |> DefaultCameraController.control win.Mouse win.Keyboard win.Time
    let proj = win.Sizes |> AVal.map (fun s -> Frustum.perspective 60.0 0.1 100.0 (float s.X / float s.Y))

    let instanceTrafos = 
        Things.trafos
        |> AVal.map (fun arr -> 
            arr |> Array.map (fun struct(r,t) -> 
                let t : Trafo3d = Euclidean3d(r,t) |> Euclidean3d.op_Explicit
                t
            )
        )

    use bufferPool = new Memory.BufferPool();
    let narrowPhaseCb = NarrowPhaseCallbacks()
    let poseIntegratorCb = MyCallbacks()
    let desc = SolveDescription(8,1)
    let sim = Simulation.Create(bufferPool, narrowPhaseCb, poseIntegratorCb, desc)
    

    // INIT
    let b = Collidables.Box(0.5f,0.5f,0.5f)
    let bCollider = sim.Shapes.Add(&b)
    for struct(r,t) in Things.trafos.Value do 
        let bInertia = b.ComputeInertia(1.0f)
        let pos = Vector3(float32 t.X, float32 t.Y, float32 t.Z)
        let rot = Quaternion(float32 r.X, float32 r.Y, float32 r.Z, float32 r.W)
        let bPose = RigidPose(pos,rot)
        let bDesc = BodyDescription.CreateDynamic(bPose, bInertia, bCollider, 0.01f)
        sim.Bodies.Add(&bDesc) |> ignore
        
    let bound = Collidables.Box(50.0f,50.0f,50.0f)
    let boundCollider = sim.Shapes.Add(&bound)
    let boundDesc = StaticDescription(Vector3(0.0f,0.0f,0.0f),boundCollider)
    sim.Statics.Add(&boundDesc) |> ignore


    // RUN
    use disp = new ThreadDispatcher(Environment.ProcessorCount)
    async {
        let mutable last = None
        while true do
            let t = DateTime.Now
            match last with 
            | Some lt -> 
                last <- Some t 
                let dt = (t - lt).Milliseconds
                sim.Timestep(float32 dt)//, disp)
                do! Async.Sleep(4)
                printfn "step %d" dt
            | None -> 
                last <- Some t
                do! Async.Sleep(10)
                printfn "no step"
    } |> Async.Start

    let sgObjects =
        Sg.box' C4b.Red (Box3d.FromCenterAndSize(V3d.OOO, V3d.III))
            |> Sg.instanced instanceTrafos
            |> Sg.effect [
                DefaultSurfaces.trafo |> toEffect
                DefaultSurfaces.vertexColor |> toEffect
                DefaultSurfaces.simpleLighting |> toEffect
               ]

    let sgBound =
        Sg.wireBox' C4b.Red (Box3d.FromCenterAndSize(V3d.OOO, V3d.III * 50.0))
        |> Sg.shader {
            do! DefaultSurfaces.trafo
            do! DefaultSurfaces.vertexColor
        }




    let sg = 
        Sg.ofList [
            sgObjects
            sgBound
        ] |> Sg.viewTrafo (view |> AVal.map CameraView.viewTrafo)
          |> Sg.projTrafo (proj |> AVal.map Frustum.projTrafo)

    let task =
        app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- task
    win.Run()
    0