@@
identifier I2CCLK_MAXMIN =~ "^I2CCLK.*_(MAX|MIN)$";
expression E;
@@

- #define I2CCLK_MAXMIN E

@@
typedef uint32_t;
@@

#define I2C_ADDFORMAT_10BITS I2C_SADDR0_ADDFORMAT /*!< address:10 bits */
+ 
+ /* I2C clock frequency, MHz */
+ #define I2CCLK_MAX                    ((uint32_t)0x0000003FU)                  /*!< i2cclk maximum value */
+ #define I2CCLK_MIN                    ((uint32_t)0x00000002U)                  /*!< i2cclk minimum value for standard mode */
+ #define I2CCLK_FM_MIN                 ((uint32_t)0x00000008U)                  /*!< i2cclk minimum value for fast mode */
+ #define I2CCLK_FM_PLUS_MIN            ((uint32_t)0x00000018U)                  /*!< i2cclk minimum value for fast mode plus */
