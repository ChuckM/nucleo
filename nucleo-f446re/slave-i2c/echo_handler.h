/*
 * Echo handler
 *
 * This simple handler stores bytes that are written to the device
 * and returns them when the device is read.
 *
 * Note, this is dynamically allocated because that makes it "easy"
 * to have two echo responders, but mostly it also just makes the
 * code more flexible.
 */

struct echo_state_t {
	ringbuffer_t	*rb;
	uint32_t		bytes_sent, bytes_recv;
	uint32_t		send_errs, recv_errs;
	uint32_t		send_count, receive_count, err_count;
	uint32_t		total;
	uint32_t		processed;
};

/* Prototypes */
struct echo_state_t *echo_init(uint16_t buf_size);
extern const i2c_handler_t echo_handler;

