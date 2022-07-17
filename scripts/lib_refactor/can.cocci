@@
identifier CAN_XXX_MODE =~ "^CAN_.*_MODE$";
fresh identifier GD32_CAN_XXX_MODE = "GD32_" ## CAN_XXX_MODE;
@@

- CAN_XXX_MODE
+ GD32_CAN_XXX_MODE

@@
identifier CAN_XXX_MODE =~ "^CAN_.*_MODE$";
fresh identifier GD32_CAN_XXX_MODE = "GD32_" ## CAN_XXX_MODE;
@@

- #define CAN_XXX_MODE
+ #define GD32_CAN_XXX_MODE

@@
@@

- CAN_TIMEOUT
+ GD32_CAN_TIMEOUT

@@
@@

- #define CAN_TIMEOUT
+ #define GD32_CAN_TIMEOUT
