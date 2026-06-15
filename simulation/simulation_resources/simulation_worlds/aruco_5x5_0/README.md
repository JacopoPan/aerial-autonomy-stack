## Spawn One Marker

Run this inside the simulation container, remember to change $WORLD_NAME accordingly

```sh

gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "/aas/simulation_resources/simulation_worlds/aruco_5x5_0/model.sdf", name: "aruco_0", allow_renaming: true, pose: {position: {x: 10, y: -20, z: 2.0}}'
```

## Spawn Several Markers

Run this inside the simulation container, remember to change $WORLD_NAME accordingly

```sh

for spec in \
  "aruco_0 10 -20 2.0" \
  "aruco_1 11 -20 2.0" \
  "aruco_2 10 -19 2.0" \
  "aruco_3 11 -19 2.0"
do
  set -- $spec
  gz service -s /world/$WORLD_NAME/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 3000 \
    --req "sdf_filename: \"/aas/simulation_resources/simulation_worlds/aruco_5x5_0/model.sdf\", name: \"$1\", allow_renaming: true, pose: {position: {x: $2, y: $3, z: $4}}"
done
```

## List Or Remove

List spawned models:

```sh
gz model --list
```