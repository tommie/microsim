#include "execution.h"

#include "../util/status.h"
#include "errors.h"

using sim::util::Status;

namespace sim::pic14::internal {

  inline void Executor::StatusRegImpl::update_add(uint8_t a, uint8_t b) {
    set_masked<(1 << C) | (1 << DC) | (1 << Z)>((static_cast<uint16_t>(a) + b > 0xFF ? (1 << C) : 0)
                                                | ((a & 0x0F) + (b & 0x0F) > 0xF ? (1 << DC) : 0)
                                                | (a + b == 0 ? (1 << Z) : 0));
  }

  inline void Executor::StatusRegImpl::update_sub(uint8_t a, uint8_t b) {
    set_masked<(1 << C) | (1 << DC) | (1 << Z)>((b <= a ? (1 << C) : 0)
                                                | ((b & 0x0F) <= (a & 0x0F) ? (1 << DC) : 0)
                                                | (a == b ? (1 << Z) : 0));
  }

  inline void Executor::StatusRegImpl::update_logic(uint8_t v) {
    set_bit<Z>(v == 0);
  }

  Executor::Executor(sim::core::DeviceListener *listener, sim::core::Clock *fosc, NonVolatile *nv, DataBus &&data_bus, InterruptMux *interrupt_mux)
    : listener_(listener),
      fosc_(fosc),
      nv_(nv),
      data_bus_(std::move(data_bus)),
      interrupt_mux_(interrupt_mux),
      status_reg_(SingleRegisterBackend<uint8_t>((1u << StatusReg::PD) | (1u << StatusReg::TO))) {}

  void Executor::reset() {
    status_reg_.reset();

    sp_reg_ = 0;
    w_reg_ = 0;
    in_sleep = false;

    schedule_immediately();
  }

  inline uint16_t Executor::pop_stack(Executor::StackContext context) {
    if (sp_reg_ == 0) {
      listener_->invalid_internal_state(Status(Error::stack_underflow, stack_context_text(context)));
    }
    sp_reg_ = (sp_reg_ + stack.size() - 1) % stack.size();
    return stack[sp_reg_];
  }

  inline void Executor::push_stack(uint16_t pc, Executor::StackContext context) {
    stack[sp_reg_] = pc;
    sp_reg_ = (sp_reg_ + 1) % stack.size();
    if (sp_reg_ == 0) {
      listener_->invalid_internal_state(Status(Error::stack_overflow, stack_context_text(context)));
    }
  }

  sim::core::Advancement Executor::advance_to(const sim::core::SimulationLimit &limit) {
    sim::core::Ticks next_tick = fosc_->at(0);

    while (limit.end_tick < 0 || next_tick - limit.end_tick <= 0) {
      next_tick = fosc_->at(execute() * TICKS_PER_INSN);

      if (limit.cond && !limit.cond(next_tick)) break;
    }

    return {
      .at_tick = next_tick,
      .next_tick = in_sleep ? -1 : next_tick,
    };
  }

  sim::core::Ticks Executor::execute() {
    if (in_sleep) {
      return 0;
    }

    uint16_t pc = get_pc();
    uint16_t insn = nv_->progmem()[pc];
    set_pc(pc + 1);

    switch (insn & 0x3800) {
    case 0x0000:
      switch (insn & 0x0700) {
      case 0x0000:
        switch (insn & 0x0080) {
        case 0x0000:
          switch (insn & 0x007F) {
          case 0x0000:
          case 0x0020:
          case 0x0040:
          case 0x0060:
            // nop
            break;

          case 0x0064:
            // clrwdt: to, pd
            // TODO: clear WDT
            status_reg_.set_reset(0);
            break;

          case 0x0009:
            // retfie
            set_pc(pop_stack(RETFIE));
            interrupt_mux_->intcon_reg().set_gie(true);
            break;

          case 0x0008:
            // return
            set_pc(pop_stack(RETURN));
            break;

          case 0x0063:
            // sleep: to, pd
            if (!interrupt_mux_->is_active()) {
              in_sleep = true;
              status_reg_.set_reset((1 << StatusReg::PD));
            }
            break;
          }
          break;

        case 0x0080:
          // movwf
          set_register(banked_data_addr(insn & 0x007F), get_w());
          break;
        }
        break;

      case 0x0100:
        switch (insn & 0x0080) {
        case 0x0080:
          // clrf: z
          set_register(banked_data_addr(insn & 0x007F), 0);
          status_reg_.update_logic(0);
          break;

        case 0x0000:
          // clrw: z
          set_w(0);
          status_reg_.update_logic(0);
          break;
        }
        break;

      default: {
        bool d_f = (insn & 0x80) == 0x80;
        uint16_t f = banked_data_addr(insn & 0x007F);
        uint8_t v = get_register(f);

        switch (insn & 0x0700) {
        case 0x0200: {
          // subwf: c, dc, z
          uint8_t w = get_w();
          status_reg_.update_sub(v, w);
          v -= w;
          break;
        }
        case 0x0300:
          // decf: z
          --v;
          status_reg_.update_logic(v);
          break;

        case 0x0400:
          // iorwf: z
          v |= get_w();
          status_reg_.update_logic(v);
          break;

        case 0x0500:
          // andwf: z
          v &= get_w();
          status_reg_.update_logic(v);
          break;

        case 0x0600:
          // xorwf: z
          v ^= get_w();
          status_reg_.update_logic(v);
          break;

        case 0x0700: {
          // addwf: c, dc, z
          uint8_t w = get_w();
          status_reg_.update_add(v, w);
          v += w;
          break;
        }
        }
        if (d_f) {
          set_register(f, v);
        } else {
          set_w(v);
        }
        break;
      }
      }
      break;

    case 0x0800: {
      bool d_f = (insn & 0x80) == 0x80;
      uint16_t f = banked_data_addr(insn & 0x007F);
      uint8_t v = get_register(f);

      switch (insn & 0x0700) {
      case 0x0000:
        // movf: z
        status_reg_.update_logic(v);
        break;

      case 0x0100:
        // comf: z
        v = ~v;
        status_reg_.update_logic(v);
        break;

      case 0x0200:
        // incf: z
        ++v;
        status_reg_.update_logic(v);
        break;

      case 0x0300:
        // decfsz
        --v;
        if (v == 0) {
          set_pc(get_pc() + 1);
        }
        break;

      case 0x0400: {
        // rrf: c
        uint8_t c = status_reg_.c() ? 0x80 : 0;
        status_reg_.set_c(v & 0x01);
        v = (v >> 1) | c;
        break;
      }
      case 0x0500: {
        // rlf: c
        uint8_t c = status_reg_.c() ? 1 : 0;
        status_reg_.set_c(v & 0x80);
        v = (v << 1) | c;
        break;
      }
      case 0x0600:
        // swapf
        v = ((v & 0x0F) << 4) | ((v >> 4) & 0x0F);
        break;

      case 0x0700:
        // incfsz
        ++v;
        if (v == 0) {
          set_pc(get_pc() + 1);
        }
        break;
      }
      if (d_f) {
        set_register(f, v);
      } else {
        set_w(v);
      }
      break;
    }
    case 0x1000: {
      uint8_t b = (insn >> 7) & 7;
      uint16_t f = banked_data_addr(insn & 0x007F);
      uint8_t v = get_register(f);

      switch (insn & 0x0400) {
      case 0x0000:
        // bcf
        v &= ~(1 << b);
        break;

      case 0x0400:
        // bsf
        v |= (1 << b);
        break;
      }
      set_register(f, v);
      break;
    }
    case 0x1800: {
      uint8_t b = (insn >> 7) & 7;
      uint16_t f = banked_data_addr(insn & 0x007F);
      bool v = (get_register(f) & (1 << b)) != 0;

      switch (insn & 0x0400) {
      case 0x0000:
        // btfsc
        v = !v;
        break;

      case 0x0400:
        // btfss
        break;
      }
      if (v) {
        set_pc(get_pc() + 1);
      }
      break;
    }
    case 0x2000: {
      // call
      uint16_t pc = get_pc();
      push_stack(pc + 1, CALL);
      set_pc((pc & 0x1800) | (insn & 0x07FF));
      break;
    }
    case 0x2800:
      // goto
      set_pc((get_pc() & 0x1800) | (insn & 0x07FF));
      break;

    case 0x3000:
      switch (insn & 0x0400) {
      case 0x0000:
        // movlw
        set_w(insn & 0x00FF);
        break;

      case 0x0400:
        // retlw
        set_w(insn & 0x00FF);
        set_pc(pop_stack(RETLW));
        break;
      }
      break;

    case 0x3800:
      switch (insn & 0x0700) {
      case 0x0000: {
        // iorlw: z
        uint8_t v = get_w() | (insn & 0x00FF);
        set_w(v);
        status_reg_.update_logic(v);
        break;
      }
      case 0x0100: {
        // andlw: z
        uint8_t v = get_w() & (insn & 0x00FF);
        set_w(v);
        status_reg_.update_logic(v);
        break;
      }
      case 0x0200: {
        // xorlw: z
        uint8_t v = get_w() ^ (insn & 0x00FF);
        set_w(v);
        status_reg_.update_logic(v);
        break;
      }
      case 0x0300:
        // invalid
        break;

      case 0x0400:
      case 0x0500: {
        // sublw: c, dc, z
        uint8_t w = get_w();
        uint8_t l = insn & 0x00FF;
        status_reg_.update_sub(l, w);
        set_w(l - w);
        break;
      }
      case 0x0600:
      case 0x0700: {
        // addlw: c, dc, z
        uint8_t w = get_w();
        uint8_t l = insn & 0x00FF;
        status_reg_.update_add(l, w);
        set_w(l + w);
        break;
      }
      }
      break;
    }

    return get_pc() != pc + 1 ? 2 : 1;
  }

  void Executor::interrupted() {
    auto intcon = interrupt_mux_->intcon_reg();

    if (intcon.gie()) {
      push_stack(get_pc(), INTERRUPT);
      set_pc(4);
      intcon.set_gie(false);
    }

    in_sleep = false;

    schedule_immediately();
  }

  uint8_t Executor::read_register(uint16_t addr) {
    switch (addr) {
    case 0x03: return status_reg_.read();
    default: return 0;
    }
  }

  void Executor::write_register(uint16_t addr, uint8_t value) {
    switch (addr) {
    case 0x03: status_reg_.write(value); break;
    }
  }

  std::string_view Executor::stack_context_text(Executor::StackContext context) {
    switch (context) {
    case Executor::INTERRUPT: return "interrupt";
    case Executor::CALL: return "call";
    case Executor::RETFIE: return "retfie";
    case Executor::RETURN: return "return";
    case Executor::RETLW: return "retlw";
    default: return "";
    }
  }

}  // namespace sim::pic14::internal
