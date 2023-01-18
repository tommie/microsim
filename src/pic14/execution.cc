#include "execution.h"

#include "../util/status.h"
#include "errors.h"

using sim::util::Status;

namespace sim::pic14::internal {

  void StatusReg::reset(uint8_t inv_v) {
    write(~inv_v & ((1 << TO) | (1 << PD)));
  }

  inline void StatusReg::update_reset(uint8_t inv_v) {
    set_masked<(1 << TO) | (1 << PD)>(~inv_v);
  }

  inline void StatusReg::update_add(uint8_t a, uint8_t b) {
    set_masked<(1 << C) | (1 << DC) | (1 << Z)>((static_cast<uint16_t>(a) + b > 0xFF ? (1 << C) : 0)
                                                | ((a & 0x0F) + (b & 0x0F) > 0xF ? (1 << DC) : 0)
                                                | (a + b == 0 ? (1 << Z) : 0));
  }

  inline void StatusReg::update_sub(uint8_t a, uint8_t b) {
    set_masked<(1 << C) | (1 << DC) | (1 << Z)>((b <= a ? (1 << C) : 0)
                                                | ((b & 0x0F) <= (a & 0x0F) ? (1 << DC) : 0)
                                                | (a == b ? (1 << Z) : 0));
  }

  inline void StatusReg::update_logic(uint8_t v) {
    set_bit<Z>(v == 0);
  }

  Executor::Executor(sim::core::DeviceListener *listener, sim::core::Clock *clock, sim::core::ClockScheduler *clocks, NonVolatile *nv, DataBus &&data_bus, InterruptMux *interrupt_mux)
    : listener_(listener),
      clock_(clock),
      clocks_(clocks),
      nv_(nv),
      data_bus_(std::move(data_bus)),
      interrupt_mux_(interrupt_mux) {}

  void Executor::reset(uint8_t status) {
    status_reg().reset(status);
    intcon_reg().reset();

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
    sim::core::Ticks at_tick = clock_->at(0);

    while (limit.end_tick < 0 || at_tick - limit.end_tick <= 0) {
      sim::core::Ticks n = execute();

      at_tick = clock_->at(n);

      // Reading e.g. TMR0 is based on clocks, so they need to be
      // advanced for every instruction.
      clocks_->advance_to(at_tick);

      if (limit.cond && !limit.cond(at_tick)) break;
    }

    if (nv_->is_in_icsp() || in_sleep) {
      return {.at_tick = at_tick, .next_tick = -1};
    }

    return {.at_tick = at_tick, .next_tick = clock_->at(1)};
  }

  sim::core::Ticks Executor::execute() {
    if (nv_->is_in_icsp()) {
      return 0;
    }

    if (interrupt_mux_->is_active()) {
      auto intcon = intcon_reg();

      if (intcon.gie()) {
        push_stack(get_pc(), INTERRUPT);
        set_pc(4);
        intcon.set_gie(false);
      }

      in_sleep = false;
    }

    if (in_sleep) {
      return 0;
    }

    uint16_t pc = get_pc();
    uint16_t insn = nv_->progmem()[pc];
    set_pc(pc + 1);

    auto status = status_reg();

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
            status.update_reset(0);
            break;

          case 0x0009:
            // retfie
            set_pc(pop_stack(RETFIE));
            intcon_reg().set_gie(true);
            break;

          case 0x0008:
            // return
            set_pc(pop_stack(RETURN));
            break;

          case 0x0063:
            // sleep: to, pd
            if (!interrupt_mux_->is_active()) {
              in_sleep = true;
              status.update_reset((1 << StatusReg::PD));
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
          status.update_logic(0);
          break;

        case 0x0000:
          // clrw: z
          set_w(0);
          status.update_logic(0);
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
          status.update_sub(v, w);
          v -= w;
          break;
        }
        case 0x0300:
          // decf: z
          --v;
          status.update_logic(v);
          break;

        case 0x0400:
          // iorwf: z
          v |= get_w();
          status.update_logic(v);
          break;

        case 0x0500:
          // andwf: z
          v &= get_w();
          status.update_logic(v);
          break;

        case 0x0600:
          // xorwf: z
          v ^= get_w();
          status.update_logic(v);
          break;

        case 0x0700: {
          // addwf: c, dc, z
          uint8_t w = get_w();
          status.update_add(v, w);
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
        status.update_logic(v);
        break;

      case 0x0100:
        // comf: z
        v = ~v;
        status.update_logic(v);
        break;

      case 0x0200:
        // incf: z
        ++v;
        status.update_logic(v);
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
        uint8_t c = status.c() ? 0x80 : 0;
        status.set_c(v & 0x01);
        v = (v >> 1) | c;
        break;
      }
      case 0x0500: {
        // rlf: c
        uint8_t c = status.c() ? 1 : 0;
        status.set_c(v & 0x80);
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
        status.update_logic(v);
        break;
      }
      case 0x0100: {
        // andlw: z
        uint8_t v = get_w() & (insn & 0x00FF);
        set_w(v);
        status.update_logic(v);
        break;
      }
      case 0x0200: {
        // xorlw: z
        uint8_t v = get_w() ^ (insn & 0x00FF);
        set_w(v);
        status.update_logic(v);
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
        status.update_sub(l, w);
        set_w(l - w);
        break;
      }
      case 0x0600:
      case 0x0700: {
        // addlw: c, dc, z
        uint8_t w = get_w();
        uint8_t l = insn & 0x00FF;
        status.update_add(l, w);
        set_w(l + w);
        break;
      }
      }
      break;
    }

    return get_pc() != pc + 1 ? 2 : 1;
  }

  void Executor::interrupted() {
    schedule_immediately();
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
