import { createComponent, numberArrayProp } from "@frc-web-components/app";

interface ReefFaceProps {
  setProperty: (property: string, value: unknown) => unknown;
  values:number[];
  faceDisabled: number[];
  name:string;
  index:number;
}

function ReefFace(props: ReefFaceProps) {
  const angle = (props.index*Math.PI/3) + Math.PI / 6;
  const x = 155*Math.cos(angle);
  const y = 155*Math.sin(angle);
  const checkbox_x = 50*Math.cos(angle);
  const checkbox_y = 50*Math.sin(angle);
  const faceDisabled = props.faceDisabled ?? [0,0,0,0,0,0];
  const singleFaceDisabled = !!faceDisabled[props.index];
  return (
    <div style = {{
    }}>
         <div style = {{
            width: "100px",
            height: "50px",
            // border: "4px solid pink",
            display: "flex",
            justifyContent: "center",
            gridArea: "header",
            position: "absolute",
            top: "78px",
            transform: `translate(${checkbox_x}px,${checkbox_y}px) rotate(${angle - Math.PI / 2}rad)`
          }}>
            <input
            onClick={() => {
              const newFaceDisabled = [...faceDisabled]
              const newDisable = !singleFaceDisabled
              newFaceDisabled[props.index] = newDisable ? 1 : 0;
              props.setProperty("faceDisabled", newFaceDisabled);
              // for (let value in props.values) {
              //   if (value == "0") { 
              //       props.setProperty(props.name, [1, 1, 1, 1, 1, 1]);
              //       console.log(value);
              //       return;
              //   } else {
              //     console.log("else ran" + props.values);
              //   }
              // }
              // console.log("value is 1");
              // props.setProperty(props.name, [0, 0, 0, 0, 0, 0]);
              // console.log(e.currentTarget.value)

            }}


            style={{
              width: "25px",
              height: "25px",
            }}

              type="checkbox"/>
          </div>
          
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
          transform: `translate(${x}px,${y}px) rotate(${angle - Math.PI / 2}rad)`,
          // gridTemplateAreas:
          //   "header header"
      }}
    >
       

      {(props.values ?? []).map((value, index) => {
        return (
          <button
          style={{
            width: 50,
            height: 50,
            backgroundColor: props.faceDisabled[props.index] === 1 ? value === 0 ? "gray" : "darkgreen" : value === 0 ? "lightgray" : "lime",
            borderRadius: "50%",
            fontSize: 30,
            
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
      faceDisabled: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
    },
  },
  ({ frontLeft, backLeft, backMiddle, backRight, frontRight, frontMiddle, faceDisabled,  setProperty }) => {
    return (
      <div style={{
        position: "relative",
      }}>
        <ReefFace values={frontLeft} setProperty={setProperty} name={"frontLeft"} index={2} faceDisabled={faceDisabled} />
        <ReefFace values={backLeft} setProperty={setProperty} name={"backLeft"} index={3} faceDisabled={faceDisabled}/>
        <ReefFace values={backMiddle} setProperty={setProperty} name={"backMiddle"} index={4} faceDisabled={faceDisabled}/>
        <ReefFace values={backRight} setProperty={setProperty} name={"backRight"} index={5} faceDisabled={faceDisabled}/>
        <ReefFace values={frontRight} setProperty={setProperty} name={"frontRight"} index={0} faceDisabled={faceDisabled}/>
        <ReefFace values={frontMiddle} setProperty={setProperty} name={"frontMiddle"} index={1} faceDisabled={faceDisabled}/>
      </div>
    );
  }
);
