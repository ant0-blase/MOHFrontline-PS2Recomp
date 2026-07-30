// Upper instructions (FMAC pipeline)
// ============================================================================
void VU1Interpreter::execUpper(uint32_t instr)
{
    uint8_t dest = DEST(instr);
    uint8_t ft = FT(instr);
    uint8_t fs = FS(instr);
    uint8_t fd = FD(instr);
    uint8_t op = instr & 0x3F;

    float *vd = m_state.vf[fd];
    const float *vs = m_state.vf[fs];
    const float *vt = m_state.vf[ft];
    float result[4];

    // Upper opcode decoding (bits 5:0 of upper word)
    switch (op)
    {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03: // ADDbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07: // SUBbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x08:
    case 0x09:
    case 0x0A:
    case 0x0B: // MADDbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x0C:
    case 0x0D:
    case 0x0E:
    case 0x0F: // MSUBbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x10:
    case 0x11:
    case 0x12:
    case 0x13: // MAXbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > bc) ? vs[c] : bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x14:
    case 0x15:
    case 0x16:
    case 0x17: // MINIbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < bc) ? vs[c] : bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B: // MULbc
    {
        float bc = broadcast(vt, op & 3);
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * bc;
        applyDest(vd, result, dest);
        return;
    }
    case 0x1C: // MULq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x1D: // MAXi
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > m_state.i) ? vs[c] : m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x1E: // MULi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x1F: // MINIi
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < m_state.i) ? vs[c] : m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x20: // ADDq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x21: // MADDq
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x22: // ADDi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x23: // MADDi
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x24: // SUBq
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x25: // MSUBq
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * m_state.q;
        applyDest(vd, result, dest);
        return;
    case 0x26: // SUBi
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x27: // MSUBi
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * m_state.i;
        applyDest(vd, result, dest);
        return;
    case 0x28: // ADD
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] + vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x29: // MADD
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] + vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2A: // MUL
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2B: // MAX
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] > vt[c]) ? vs[c] : vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2C: // SUB
        for (int c = 0; c < 4; c++)
            result[c] = vs[c] - vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2D: // MSUB
        for (int c = 0; c < 4; c++)
            result[c] = m_state.acc[c] - vs[c] * vt[c];
        applyDest(vd, result, dest);
        return;
    case 0x2E: // OPMSUB
        result[0] = m_state.acc[0] - vs[1] * vt[2];
        result[1] = m_state.acc[1] - vs[2] * vt[0];
        result[2] = m_state.acc[2] - vs[0] * vt[1];
        result[3] = 0.0f;
        applyDest(vd, result, dest);
        return;
    case 0x2F: // MINI
        for (int c = 0; c < 4; c++)
            result[c] = (vs[c] < vt[c]) ? vs[c] : vt[c];
        applyDest(vd, result, dest);
        return;

    // Upper special group (low op 0x3C..0x3F).
    // Like lower1 special, the real selector is not just bits 5:0.  Dobie decodes:
    //   op = (instr & 0x3) | ((instr >> 4) & 0x7C)
    // Several instructions in this group also use FT as the destination, not FD.
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
    {
        const uint8_t specialOp = static_cast<uint8_t>((instr & 0x3u) | ((instr >> 4) & 0x7Cu));
        float *vtDest = m_state.vf[ft];

        switch (specialOp)
        {
        case 0x00:
        case 0x01:
        case 0x02:
        case 0x03: // ADDAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07: // SUBAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B: // MADDAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x0C:
        case 0x0D:
        case 0x0E:
        case 0x0F: // MSUBAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x10: // ITOF0
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x11: // ITOF4
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 16.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x12: // ITOF12
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 4096.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x13: // ITOF15
            for (int c = 0; c < 4; c++)
            {
                int32_t iv;
                std::memcpy(&iv, &vs[c], 4);
                result[c] = static_cast<float>(iv) / 32768.0f;
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x14: // FTOI0
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = static_cast<int32_t>(vs[c]);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x15: // FTOI4
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = static_cast<int32_t>(vs[c] * 16.0f);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x16: // FTOI12
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = static_cast<int32_t>(vs[c] * 4096.0f);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x17: // FTOI15
            for (int c = 0; c < 4; c++)
            {
                int32_t iv = static_cast<int32_t>(vs[c] * 32768.0f);
                std::memcpy(&result[c], &iv, 4);
            }
            applyDest(vtDest, result, dest);
            return;
        case 0x18:
        case 0x19:
        case 0x1A:
        case 0x1B: // MULAbc
        {
            float bc = broadcast(vt, specialOp & 3);
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * bc;
            applyDestAcc(result, dest);
            return;
        }
        case 0x1C: // MULAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x1D: // ABS
            for (int c = 0; c < 4; c++)
                result[c] = std::fabs(vs[c]);
            applyDest(vtDest, result, dest);
            return;
        case 0x1E: // MULAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x1F: // CLIP
        {
            float w = std::fabs(vt[3]);
            uint32_t flags = 0;
            if (vs[0] > +w) flags |= 0x01;
            if (vs[0] < -w) flags |= 0x02;
            if (vs[1] > +w) flags |= 0x04;
            if (vs[1] < -w) flags |= 0x08;
            if (vs[2] > +w) flags |= 0x10;
            if (vs[2] < -w) flags |= 0x20;
            m_state.clip = (m_state.clip << 6) | flags;
            return;
        }
        case 0x20: // ADDAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x21: // MADDAq
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x22: // ADDAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x23: // MADDAi
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x24: // SUBAq
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x25: // MSUBAq
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * m_state.q;
            applyDestAcc(result, dest);
            return;
        case 0x26: // SUBAi
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x27: // MSUBAi
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * m_state.i;
            applyDestAcc(result, dest);
            return;
        case 0x28: // ADDA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] + vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x29: // MADDA
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] + vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2A: // MULA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2C: // SUBA
            for (int c = 0; c < 4; c++)
                result[c] = vs[c] - vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2D: // MSUBA
            for (int c = 0; c < 4; c++)
                result[c] = m_state.acc[c] - vs[c] * vt[c];
            applyDestAcc(result, dest);
            return;
        case 0x2E: // OPMULA
            result[0] = vs[1] * vt[2];
            result[1] = vs[2] * vt[0];
            result[2] = vs[0] * vt[1];
            result[3] = 0.0f;
            applyDestAcc(result, dest);
            return;
        case 0x2F:
        case 0x30: // NOP
            return;
        default:
            return;
        }
    }

    case 0x30:
    case 0x31:
    case 0x32:
    case 0x33:
    default:
        return;
    }
}

// ============================================================================
// Lower instructions
// ============================================================================
void VU1Interpreter::execLower(uint32_t instr, uint8_t *vuData, uint32_t dataSize, GS &gs, PS2Memory *memory, uint32_t upperInstr)
{
    (void)upperInstr;
    if (instr == 0x00000000 || instr == 0x8000033C) // NOP
        return;

    uint8_t opHi = (instr >> 25) & 0x7F;

    // The lower instruction encoding uses bits 31:25 for the primary opcode
    switch (opHi)
    {
    case 0x00: // LQ (Load Quadword from VU data memory)
    {
        uint8_t it = FT(instr);      // VF destination
        uint8_t is = VIS(instr);    // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            float tmp[4];
            std::memcpy(tmp, vuData + addr, 16);
            applyDest(m_state.vf[it], tmp, dest);
        }
        return;
    }
    case 0x01: // SQ (Store Quadword to VU data memory)
    {
        uint8_t is = FS(instr);      // VF source
        uint8_t it = VIT(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[it] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            float tmp[4];
            std::memcpy(tmp, vuData + addr, 16);
            if (dest & 0x8)
                tmp[0] = m_state.vf[is][0];
            if (dest & 0x4)
                tmp[1] = m_state.vf[is][1];
            if (dest & 0x2)
                tmp[2] = m_state.vf[is][2];
            if (dest & 0x1)
                tmp[3] = m_state.vf[is][3];
            std::memcpy(vuData + addr, tmp, 16);
        }
        return;
    }
    case 0x04: // ILW (Integer Load Word from VU data memory)
    {
        uint8_t it = VIT(instr);     // VI destination
        uint8_t is = VIS(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            int comp = 0;
            if (dest & 0x8)
                comp = 0;
            else if (dest & 0x4)
                comp = 1;
            else if (dest & 0x2)
                comp = 2;
            else
                comp = 3;
            uint32_t v;
            std::memcpy(&v, vuData + addr + comp * 4, 4);
            if (it != 0)
                m_state.vi[it] = (int32_t)(int16_t)(v & 0xFFFF);
        }
        return;
    }
    case 0x05: // ISW (Integer Store Word to VU data memory)
    {
        uint8_t it = VIT(instr);     // VI source
        uint8_t is = VIS(instr);     // VI base
        uint8_t dest = (instr >> 21) & 0xF;
        int16_t imm = IMM11(instr);
        uint32_t addr = ((uint32_t)(int32_t)(m_state.vi[is] + imm)) * 16u;
        addr &= (dataSize - 1);
        if (addr + 16 <= dataSize)
        {
            uint32_t val = (uint32_t)(uint16_t)(m_state.vi[it] & 0xFFFF);
            if (dest & 0x8)
                std::memcpy(vuData + addr + 0, &val, 4);
            if (dest & 0x4)
                std::memcpy(vuData + addr + 4, &val, 4);
            if (dest & 0x2)
                std::memcpy(vuData + addr + 8, &val, 4);
            if (dest & 0x1)
                std::memcpy(vuData + addr + 12, &val, 4);
        }
        return;
    }
    case 0x08: // IADDIU
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = (int16_t)(instr & 0x7FF) | ((instr >> 10) & 0x7800);
        if (it != 0)
            m_state.vi[it] = (int16_t)(m_state.vi[is] + imm);
        return;
    }
    case 0x09: // ISUBIU
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = (int16_t)(instr & 0x7FF) | ((instr >> 10) & 0x7800);
        if (it != 0)
            m_state.vi[it] = (int16_t)(m_state.vi[is] - imm);
        return;
    }
    case 0x10: // FCEQ
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip & 0xFFFFFF) == imm24) ? 1 : 0;
        return;
    }
    case 0x11: // FCSET
    {
        m_state.clip = instr & 0xFFFFFF;
        return;
    }
    case 0x12: // FCAND
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip & imm24) != 0) ? 1 : 0;
        return;
    }
    case 0x13: // FCOR
    {
        uint32_t imm24 = instr & 0xFFFFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.clip | imm24) == 0xFFFFFF) ? 1 : 0;
        return;
    }
    case 0x14: // FSEQ
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.status & 0xFFF) == imm12) ? 1 : 0;
        return;
    }
    case 0x15: // FSSET
    {
        m_state.status = (instr >> 6) & 0xFC0;
        return;
    }
    case 0x16: // FSAND
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = (int32_t)(m_state.status & imm12);
        return;
    }
    case 0x17: // FSOR
    {
        uint16_t imm12 = instr & 0xFFF;
        if (1 != 0)
            m_state.vi[1] = ((m_state.status | imm12) == 0xFFF) ? 1 : 0;
        return;
    }
    case 0x18: // FMAND
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = (int32_t)(m_state.mac & (uint32_t)(uint16_t)m_state.vi[is]);
        return;
    }
    case 0x1A: // FMEQ
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = ((m_state.mac & 0xFFFF) == (uint32_t)(uint16_t)m_state.vi[is]) ? 1 : 0;
        return;
    }
    case 0x1C: // FMOR
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        if (it != 0)
            m_state.vi[it] = (int32_t)(m_state.mac | (uint32_t)(uint16_t)m_state.vi[is]);
        return;
    }
    case 0x20: // B (unconditional branch)
    {
        int16_t imm = IMM11(instr);
        uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x21: // BAL (Branch and link)
    {
        uint8_t it = VIT(instr);
        int16_t imm = IMM11(instr);
        uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
        if (it != 0)
            m_state.vi[it] = (int32_t)((m_state.pc + 16) / 8);
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x24: // JR
    {
        uint8_t is = VIS(instr);
        uint32_t target = ((uint32_t)(uint16_t)m_state.vi[is] * 8u) & 0x3FFF;
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x25: // JALR
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        uint32_t target = ((uint32_t)(uint16_t)m_state.vi[is] * 8u) & 0x3FFF;
        if (it != 0)
            m_state.vi[it] = (int32_t)((m_state.pc + 16) / 8);
        m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        return;
    }
    case 0x28: // IBEQ
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] == (int16_t)m_state.vi[it])
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x29: // IBNE
    {
        uint8_t it = VIT(instr);
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] != (int16_t)m_state.vi[it])
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2C: // IBLTZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] < 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2D: // IBGTZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] > 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2E: // IBLEZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] <= 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }
    case 0x2F: // IBGEZ
    {
        uint8_t is = VIS(instr);
        int16_t imm = IMM11(instr);
        if ((int16_t)m_state.vi[is] >= 0)
        {
            uint32_t target = (m_state.pc + 8 + imm * 8) & 0x3FFF;
            m_state.branchPending = true;
        m_state.branchTarget = target;
        m_state.branchDelay = 1;
        }
        return;
    }

    case 0x40: // Lower1 / lower special. Bit31 set; low 6 bits select integer or special op.
    {
        const uint8_t funct = instr & 0x3Fu;
        const uint8_t vfT = FT(instr);
        const uint8_t vfS = FS(instr);
        const uint8_t viT = VIT(instr);
        const uint8_t viS = VIS(instr);
        const uint8_t viD = VID(instr);
        const uint8_t dest = (instr >> 21) & 0xF;

        auto doXgkick = [&]()
        {
            if (!vuData || dataSize < 16u)
                return;

            auto wrapOffset = [&](uint32_t off) -> uint32_t
            {
                return off % dataSize;
            };

            auto read64Wrap = [&](uint32_t off) -> uint64_t
            {
                uint8_t bytes[8];
                for (uint32_t i = 0; i < 8u; ++i)
                {
                    bytes[i] = vuData[wrapOffset(off + i)];
                }
                uint64_t value = 0;
                std::memcpy(&value, bytes, sizeof(value));
                return value;
            };

            uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
            addr = wrapOffset(addr);
            uint32_t pktOff = addr;
            uint32_t totalBytes = 0u;
            bool done = false;

            for (int safety = 0; safety < 256 && !done; ++safety)
            {
                uint64_t tagLo = read64Wrap(pktOff);
                uint32_t nloop = (uint32_t)(tagLo & 0x7FFFu);
                uint8_t flg = (uint8_t)((tagLo >> 58) & 0x3u);
                uint32_t nreg = (uint32_t)((tagLo >> 60) & 0xFu);
                if (nreg == 0u)
                    nreg = 16u;
                bool eop = ((tagLo >> 15) & 0x1ull) != 0ull;

                uint32_t pktSize = 16u;
                if (flg == 0u)
                {
                    pktSize += nloop * nreg * 16u;
                }
                else if (flg == 1u)
                {
                    uint32_t regs = nloop * nreg;
                    pktSize += regs * 8u;
                    if ((regs & 1u) != 0u)
                        pktSize += 8u;
                }
                else if (flg == 2u)
                {
                    pktSize += nloop * 16u;
                }

                if (pktSize == 0u)
                    break;

                totalBytes += pktSize;
                pktOff = wrapOffset(pktOff + pktSize);
                if (eop)
                    done = true;
            }

            if (totalBytes == 0u)
                return;

            if (addr + totalBytes <= dataSize)
            {
                if (memory)
                    memory->submitGifPacket(GifPathId::Path1, vuData + addr, totalBytes);
                else
                    gs.processGIFPacket(vuData + addr, totalBytes);
            }
            else
            {
                std::vector<uint8_t> wrappedPacket(totalBytes);
                for (uint32_t i = 0; i < totalBytes; ++i)
                {
                    wrappedPacket[i] = vuData[wrapOffset(addr + i)];
                }

                if (memory)
                    memory->submitGifPacket(GifPathId::Path1, wrappedPacket.data(), totalBytes);
                else
                    gs.processGIFPacket(wrappedPacket.data(), totalBytes);
            }
        };

        switch (funct)
        {
        case 0x30: // IADD
            if (viD != 0)
                m_state.vi[viD] = (int16_t)(m_state.vi[viS] + m_state.vi[viT]);
            return;
        case 0x31: // ISUB
            if (viD != 0)
                m_state.vi[viD] = (int16_t)(m_state.vi[viS] - m_state.vi[viT]);
            return;
        case 0x32: // IADDI
        {
            int16_t imm5 = (int16_t)((int32_t)((instr >> 6) & 0x1F) << 27 >> 27);
            if (viT != 0)
                m_state.vi[viT] = (int16_t)(m_state.vi[viS] + imm5);
            return;
        }
        case 0x34: // IAND
            if (viD != 0)
                m_state.vi[viD] = m_state.vi[viS] & m_state.vi[viT];
            return;
        case 0x35: // IOR
            if (viD != 0)
                m_state.vi[viD] = m_state.vi[viS] | m_state.vi[viT];
            return;

        case 0x3C:
        case 0x3D:
        case 0x3E:
        case 0x3F: // Lower1 special. Dobie decodes this as (instr & 3) | ((instr >> 4) & 0x7C).
        {
            const uint8_t funct2 = (uint8_t)((instr & 0x3u) | ((instr >> 4) & 0x7Cu));
            switch (funct2)
            {
            case 0x30: // MOVE
            {
                float tmp[4];
                std::memcpy(tmp, m_state.vf[vfS], 16);
                applyDest(m_state.vf[vfT], tmp, dest);
                return;
            }
            case 0x31: // MR32 (rotate right by 32 bits = shift xyzw -> yzwx)
            {
                float tmp[4] = {m_state.vf[vfS][1], m_state.vf[vfS][2], m_state.vf[vfS][3], m_state.vf[vfS][0]};
                applyDest(m_state.vf[vfT], tmp, dest);
                return;
            }
            case 0x34: // LQI (Load Quadword, post-increment)
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    applyDest(m_state.vf[vfT], tmp, dest);
                }
                if (viS != 0)
                    m_state.vi[viS] = (int16_t)(m_state.vi[viS] + 1);
                return;
            }
            case 0x35: // SQI (Store Quadword, post-increment)
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viT]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    if (dest & 0x8)
                        tmp[0] = m_state.vf[vfS][0];
                    if (dest & 0x4)
                        tmp[1] = m_state.vf[vfS][1];
                    if (dest & 0x2)
                        tmp[2] = m_state.vf[vfS][2];
                    if (dest & 0x1)
                        tmp[3] = m_state.vf[vfS][3];
                    std::memcpy(vuData + addr, tmp, 16);
                }
                if (viT != 0)
                    m_state.vi[viT] = (int16_t)(m_state.vi[viT] + 1);
                return;
            }
            case 0x36: // LQD (Load Quadword, pre-decrement)
            {
                if (viS != 0)
                    m_state.vi[viS] = (int16_t)(m_state.vi[viS] - 1);
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    applyDest(m_state.vf[vfT], tmp, dest);
                }
                return;
            }
            case 0x37: // SQD (Store Quadword, pre-decrement)
            {
                if (viT != 0)
                    m_state.vi[viT] = (int16_t)(m_state.vi[viT] - 1);
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viT]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    float tmp[4];
                    std::memcpy(tmp, vuData + addr, 16);
                    if (dest & 0x8)
                        tmp[0] = m_state.vf[vfS][0];
                    if (dest & 0x4)
                        tmp[1] = m_state.vf[vfS][1];
                    if (dest & 0x2)
                        tmp[2] = m_state.vf[vfS][2];
                    if (dest & 0x1)
                        tmp[3] = m_state.vf[vfS][3];
                    std::memcpy(vuData + addr, tmp, 16);
                }
                return;
            }
            case 0x38: // DIV
            {
                int fsf = (instr >> 21) & 0x3;
                int ftf = (instr >> 23) & 0x3;
                float num = m_state.vf[vfS][fsf];
                float den = m_state.vf[vfT][ftf];
                if (den != 0.0f)
                    m_state.q = num / den;
                else
                    m_state.q = (num >= 0.0f) ? std::numeric_limits<float>::max() : -std::numeric_limits<float>::max();
                return;
            }
            case 0x39: // SQRT
            {
                int ftf = (instr >> 23) & 0x3;
                float val = m_state.vf[vfT][ftf];
                m_state.q = std::sqrt(std::fabs(val));
                return;
            }
            case 0x3A: // RSQRT
            {
                int fsf = (instr >> 21) & 0x3;
                int ftf = (instr >> 23) & 0x3;
                float num = m_state.vf[vfS][fsf];
                float den = std::sqrt(std::fabs(m_state.vf[vfT][ftf]));
                if (den != 0.0f)
                    m_state.q = num / den;
                else
                    m_state.q = std::numeric_limits<float>::max();
                return;
            }
            case 0x3B: // WAITQ
                return;
            case 0x3C: // MTIR (Move To Integer Register)
            {
                int comp = 0;
                if (dest & 0x8)
                    comp = 0;
                else if (dest & 0x4)
                    comp = 1;
                else if (dest & 0x2)
                    comp = 2;
                else
                    comp = 3;
                uint32_t fval;
                std::memcpy(&fval, &m_state.vf[vfS][comp], 4);
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(int16_t)(fval & 0xFFFF);
                return;
            }
            case 0x3D: // MFIR (Move From Integer Register)
            {
                float result[4];
                int32_t val = (int32_t)(int16_t)(m_state.vi[viS] & 0xFFFF);
                std::memcpy(&result[0], &val, 4);
                result[1] = result[0];
                result[2] = result[0];
                result[3] = result[0];
                applyDest(m_state.vf[vfT], result, dest);
                return;
            }
            case 0x3E: // ILWR - integer load word from address in VI[is]
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    int comp = 0;
                    if (dest & 0x8)
                        comp = 0;
                    else if (dest & 0x4)
                        comp = 1;
                    else if (dest & 0x2)
                        comp = 2;
                    else
                        comp = 3;
                    uint32_t v;
                    std::memcpy(&v, vuData + addr + comp * 4, 4);
                    if (viT != 0)
                        m_state.vi[viT] = (int32_t)(int16_t)(v & 0xFFFF);
                }
                return;
            }
            case 0x3F: // ISWR - integer store word to address in VI[is]
            {
                uint32_t addr = ((uint32_t)(uint16_t)m_state.vi[viS]) * 16u;
                addr &= (dataSize - 1);
                if (addr + 16 <= dataSize)
                {
                    uint32_t val = (uint32_t)(uint16_t)(m_state.vi[viT] & 0xFFFF);
                    if (dest & 0x8)
                        std::memcpy(vuData + addr + 0, &val, 4);
                    if (dest & 0x4)
                        std::memcpy(vuData + addr + 4, &val, 4);
                    if (dest & 0x2)
                        std::memcpy(vuData + addr + 8, &val, 4);
                    if (dest & 0x1)
                        std::memcpy(vuData + addr + 12, &val, 4);
                }
                return;
            }
            case 0x40: // RNEXT
                return;
            case 0x41: // RGET
                return;
            case 0x42: // RINIT
                return;
            case 0x43: // RXOR
                return;
            case 0x64: // MFP (Move From P register)
            {
                float result[4] = {m_state.p, m_state.p, m_state.p, m_state.p};
                applyDest(m_state.vf[vfT], result, dest);
                return;
            }
            case 0x68: // XTOP - move current VIF1 TOP into VI register
            {
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(m_state.top & 0x3FFu);
                return;
            }
            case 0x69: // XITOP - move current VIF1 ITOP into VI register
            {
                if (viT != 0)
                    m_state.vi[viT] = (int32_t)(m_state.itop & 0x3FFu);
                return;
            }
            case 0x6C: // XGKICK - send GIF packet from VU1 data memory
                doXgkick();
                return;
            case 0x70: // ESADD
                return;
            case 0x71: // ERSADD
                return;
            case 0x72: // ELENG
            {
                float s = m_state.vf[vfS][0] * m_state.vf[vfS][0] + m_state.vf[vfS][1] * m_state.vf[vfS][1] + m_state.vf[vfS][2] * m_state.vf[vfS][2];
                m_state.p = std::sqrt(s);
                return;
            }
            case 0x73: // ERLENG
            {
                float s = m_state.vf[vfS][0] * m_state.vf[vfS][0] + m_state.vf[vfS][1] * m_state.vf[vfS][1] + m_state.vf[vfS][2] * m_state.vf[vfS][2];
                float len = std::sqrt(s);
                m_state.p = (len != 0.0f) ? (1.0f / len) : std::numeric_limits<float>::max();
                return;
            }
            case 0x7A: // ERCPR
            {
                int fsf = (instr >> 21) & 0x3;
                float val = m_state.vf[vfS][fsf];
                m_state.p = (val != 0.0f) ? (1.0f / val) : std::numeric_limits<float>::max();
                return;
            }
            case 0x7B: // WAITP
                return;
            case 0x7D: // EATAN / EATANxy / EATANxz placeholder
                return;
            default:
                return;
            }
        }
        default:
            return;
        }
    }
    default:
        break;
    }
}
=======
// VU1 interpreter implementation has been split into src/lib/vu/*.cpp.
