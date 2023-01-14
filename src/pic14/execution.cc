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

  Execution::Execution(sim::core::DeviceListener *listener, sim::core::Clock *clock, const NonVolatile::Config &nv_config, DataBus &&data_bus)
    : Device(listener),
      NonVolatile(nv_config),
      clock_(clock),
      data_bus_(std::move(data_bus)) {
    reset(0);
  }

  void Execution::reset(uint8_t status) {
    Interruption::reset();

    status_reg().reset(status);
    intcon_reg().reset();

    sp_reg_ = 0;
    w_reg_ = 0;
    in_sleep = false;
  }

  inline uint16_t Execution::pop_stack(Execution::StackContext context) {
    if (sp_reg_ == 0) {
      device_listener().invalid_internal_state(Status(Error::stack_underflow, stack_context_text(context)));
    }
    sp_reg_ = (sp_reg_ + stack.size() - 1) % stack.size();
    return stack[sp_reg_];
  }

  inline void Execution::push_stack(uint16_t pc, Execution::StackContext context) {
    stack[sp_reg_] = pc;
    sp_reg_ = (sp_reg_ + 1) % stack.size();
    if (sp_reg_ == 0) {
      device_listener().invalid_internal_state(Status(Error::stack_overflow, stack_context_text(context)));
    }
  }

  sim::core::Advancement Execution::execute_to(const sim::core::SimulationLimit &limit) {
    int i = 0;

    for (sim::core::Ticks at_tick = clock_->at(0); limit.end_tick < 0 || at_tick - limit.end_tick <= 0; at_tick = clock_->at(i)) {
      i += execute();

      if (limit.cond && !limit.cond(at_tick)) break;
    }

    return {.at_tick = clock_->at(0), .next_tick = clock_->at(i)};
  }

  sim::core::Ticks Execution::execute() {
    if (is_in_icsp()) {
      return 0;
    }

    if (interrupt_mux().is_active()) {
      auto intcon = Execution::intcon_reg();

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
    uint16_t insn = progmem[pc];
    set_pc(pc + 1);

    auto status = Execution::status_reg();

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
            if (!interrupt_mux().is_active()) {
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

  std::string_view Execution::stack_context_text(Execution::StackContext context) {
    switch (context) {
    case Execution::INTERRUPT: return "interrupt";
    case Execution::CALL: return "call";
    case Execution::RETFIE: return "retfie";
    case Execution::RETURN: return "return";
    case Execution::RETLW: return "retlw";
    default: return "";
    }
  }

  sim::core::Advancement Executor::advance_to(const sim::core::SimulationLimit &limit) {
    return execution_->execute_to(limit);
  }

}  // namespace sim::pic14::internal
