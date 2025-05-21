import { createComponent, numberArrayProp } from "@frc-web-components/app";

interface ReefFaceProps {
  setProperty: (property: string, value: unknown) => unknown;
  values:number[];
  name:string;
  index:number;
}

function ReefFace(props: ReefFaceProps) {
  const angle = (props.index*Math.PI/3) + Math.PI / 6;
  const x = 155*Math.cos(angle);
  const y = 155*Math.sin(angle);

  return (
    <div
    data-face={props.name}
    style={{
      width: "100px",
      display: "grid",
      gridTemplateColumns: "repeat(2, 1fr)",
      gridTemplateRows:
        "repeat(3, auto)" /* Creates 3 rows with automatic height */,
      gap: "5px" /* Sets a 5px gap between both rows and columns */,
      position: "absolute",
      transform: `translate(${x}px,${y}px) rotate(${angle + Math.PI / 2}rad)`
    }}
  >
    {(props.values ?? []).map((value, index) => {
      return (
        <button
          style={{
            width: 50,
            height: 50,
            backgroundColor: value === 0 ? "lightgray" : "lime",
            borderRadius: "50%",
            fontSize: 30,
            fontFamily: "Copperplate"

          }}
          onClick={() => {
            const newValues = [...props.values];
            const newValue = 1 - value;
            newValues[index] = newValue;
            props.setProperty(props.name, newValues);
          }}
        >
          L{4 - Math.floor(index / 2)}
        </button>
      );
    })}
  </div>
  )
}


export const myElement = createComponent(
  {
    dashboard: {
      name: "Beef",
      description: "",
      defaultSize: { width: 130, height: 50 },
      minSize: { width: 20, height: 20 },
    },
    acceptedSourceTypes: ["Number[]"],
    primaryProperty: "count",
    properties: {
      frontLeft: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
      backLeft: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
      backMiddle: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
      backRight: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
      frontRight: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
      frontMiddle: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
    },
  },
  ({ frontLeft, backLeft, backMiddle, backRight, frontRight, frontMiddle,  setProperty }) => {
    return (
      <div style={{
        position: "relative",
      }}>
        <ReefFace values={frontLeft} setProperty={setProperty} name={"frontLeft"} index={2}/>
        <ReefFace values={backLeft} setProperty={setProperty} name={"backLeft"} index={3} />
        <ReefFace values={backMiddle} setProperty={setProperty} name={"backMiddle"} index={4}/>
        <ReefFace values={backRight} setProperty={setProperty} name={"backRight"} index={5}/>
        <ReefFace values={frontRight} setProperty={setProperty} name={"frontRight"} index={0}/>
        <ReefFace values={frontMiddle} setProperty={setProperty} name={"frontMiddle"} index={1}/>
      </div>
    );
  }
);
