import { createComponent, numberArrayProp } from "@frc-web-components/app";

export const myElement = createComponent(
  {
    dashboard: {
      name: "Reef",
      description: "",
      defaultSize: { width: 130, height: 50 },
      minSize: { width: 20, height: 20 },
    },
    acceptedSourceTypes: ["Number[]"],
    primaryProperty: "count",
    properties: {
      values: numberArrayProp({ defaultValue: [0, 0, 0, 0, 0, 0] }),
    },
  },
  ({ values, setProperty }) => {
    return (
      <div
        style={{
          width: "200px",
          display: "grid",
          gridTemplateColumns: "repeat(2, 1fr)",
          gridTemplateRows:
            "repeat(3, auto)" /* Creates 3 rows with automatic height */,
          gap: "5px" /* Sets a 5px gap between both rows and columns */,
        }}
      >
        {(values ?? []).map((value, index) => {
          return (
            <button
              style={{
                width: "95%",
                height: 100,
                backgroundColor: value === 0 ? "lightgray" : "green",
              }}
              onClick={() => {
                const newValues = [...values];
                const newValue = 1 - value;
                newValues[index] = newValue;
                setProperty("values", newValues);
              }}
            >
              L{4 - Math.floor(index / 2)}
            </button>
          );
        })}
      </div>
    );
  }
);
