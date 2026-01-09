
// first bit of id if it 0 then this the frame
typedef struct {
    uint8_t flags;
    uint32_t id;
    uint32_t lat;
    uint32_t lon;
} PositionFrame_t;