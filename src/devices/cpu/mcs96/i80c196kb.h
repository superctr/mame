// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    i80c196kb.h

    MCS96, 80C196KB: the c196 instruction set on the 8x9x peripherals

***************************************************************************/

#ifndef MAME_CPU_MCS96_I80C196KB_H
#define MAME_CPU_MCS96_I80C196KB_H

#include "i8x9x.h"

class i80c196kb_device : public i8x9x_device {
public:
	i80c196kb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void execute_set_input(int linenum, int state) override;

	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	virtual void do_exec_full() override;
	virtual void do_exec_partial() override;

	void kb_regs(address_map &map) ATTR_COLD;
	void ioc2_w(u8 data);
	virtual bool timer2_counts_down() const override;
	u8 wsr_r();
	void wsr_w(u8 data);

#define O(o) void o ## _196_full(); void o ## _196_partial()

	O(bmov_direct_2w);
	O(bmovi_direct_2w);
	O(cmpl_direct_2w);
	O(djnzw_wrrel8);
	O(idlpd_none);
	O(pop_indexed_1w);
	O(pop_indirect_1w);
	O(popa_none);
	O(pusha_none);

#undef O

private:
	u8 m_ioc2;
	bool m_extint1;
};

DECLARE_DEVICE_TYPE(I80C196KB, i80c196kb_device)

#endif // MAME_CPU_MCS96_I80C196KB_H
