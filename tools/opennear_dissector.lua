-- OpenNear v6.7 Wireshark Lua Dissector (DLT_USER0 = 147)
-- Install: Copy to Wireshark Personal Lua Plugins folder, then Edit -> Preferences -> Protocols -> DLT_USER -> User 0 -> "opennear"

local p_on = Proto("opennear", "OpenNear Clean-Slate NearLink-Class Protocol")

local f_meta_ch   = ProtoField.uint8("opennear.meta.channel", "Logical TSCH Channel", base.DEC)
local f_meta_node = ProtoField.uint8("opennear.meta.tx_node", "Sim TX Node Index", base.DEC)
local f_ver       = ProtoField.uint8("opennear.hdr.version", "Protocol Version", base.DEC, nil, 0xC0)
local f_type      = ProtoField.uint8("opennear.hdr.type", "Frame Type", base.HEX, {
    [0x0] = "SYNC_MAP",
    [0x1] = "SUB_SYNC",
    [0x2] = "TDMA_DATA",
    [0x3] = "JOIN_REQ",
    [0x4] = "JOIN_ACK",
    [0x5] = "RTT_REQ",
    [0x6] = "RTT_RESP",
    [0x7] = "CRA_ALERT",
    [0x8] = "OTA_META",
    [0x9] = "OTA_CHUNK",
    [0xA] = "OTA_STATUS",
    [0xB] = "OTA_COMMIT",
    [0xC] = "GW_BRIDGE",
    [0xD] = "MESH"
}, 0x3C)
local f_sec       = ProtoField.bool("opennear.hdr.sec_en", "AES-128-CCM Encrypted", 8, nil, 0x02)
local f_subg      = ProtoField.bool("opennear.hdr.is_sub_g", "Sub-G Window Frame", 8, nil, 0x01)
local f_src       = ProtoField.uint16("opennear.hdr.src_id", "Src Short-ID (12-bit)", base.HEX)
local f_dst       = ProtoField.uint16("opennear.hdr.dst_id", "Dst Short-ID (12-bit)", base.HEX)
local f_seq_low   = ProtoField.uint8("opennear.hdr.sf_seq_low", "SF Seq Low (4-bit)", base.DEC, nil, 0xF0)
local f_slot      = ProtoField.uint8("opennear.hdr.slot_idx", "Slot Index (4-bit)", base.DEC, nil, 0x0F)
local f_hcrc8     = ProtoField.uint8("opennear.hdr.crc8", "Header CRC-8", base.HEX)
local f_sf_seq    = ProtoField.uint32("opennear.sync.sf_seq", "Superframe Seq", base.DEC)
local f_t_tx      = ProtoField.int64("opennear.sync.t_tx_net_ns", "G TX Network Time (ns)", base.DEC)
local f_backup    = ProtoField.uint16("opennear.sync.backup_id", "Backup_ID", base.HEX)
local f_subg_id   = ProtoField.uint16("opennear.sync.sub_g_id", "Sub-G Proxy ID", base.HEX)
local f_tdma_map  = ProtoField.uint16("opennear.sync.tdma_bitmap", "TDMA Bitmap", base.HEX)
local f_ch_map    = ProtoField.uint16("opennear.sync.channel_map", "AFH Channel Map", base.HEX)
local f_ack_map   = ProtoField.uint16("opennear.sync.ack_bitmap", "Piggyback Group-ACK", base.HEX)
local f_salt      = ProtoField.uint32("opennear.sync.session_salt", "Session Salt", base.HEX)
local f_epoch     = ProtoField.uint8("opennear.sync.key_epoch", "Key Epoch", base.DEC)
local f_bmac      = ProtoField.bytes("opennear.sync.beacon_mac", "Beacon CBC-MAC (4B)")
-- v6.7 GW_BRIDGE & MESH deep fields
local f_gw_type   = ProtoField.uint8("opennear.gw.type", "GW Bridge Subtype", base.HEX, {
    [0x01] = "TELEMETRY_UP", [0x02] = "CLOUD_CMD_DOWN", [0x03] = "BLE_NOTIFY_UP",
    [0x04] = "BLE_CMD_DOWN", [0x05] = "WIFI_PROV", [0x06] = "OTA_URL_PULL",
    [0x07] = "TOPO_REQ",     [0x08] = "TOPO_RESP", [0x09] = "MAC_REPORT"
})
local f_gw_flags  = ProtoField.uint8("opennear.gw.flags", "GW Flags", base.HEX)
local f_gw_orig   = ProtoField.uint16("opennear.gw.origin_id", "Origin Short-ID", base.HEX)
local f_gw_seq    = ProtoField.uint16("opennear.gw.seq", "Bridge Seq", base.DEC)
local f_gw_plen   = ProtoField.uint8("opennear.gw.plen", "Bridge Payload Len", base.DEC)
local f_gw_mac    = ProtoField.ether("opennear.gw.mac48", "Reported Hardware MAC48")
local f_mesh_type = ProtoField.uint8("opennear.mesh.type", "Mesh Packet Type", base.HEX, {
    [0x01] = "BEACON", [0x02] = "DATA_UP", [0x03] = "CMD_DOWN", [0x04] = "TOPO_REPORT"
})
local f_mesh_hop  = ProtoField.uint8("opennear.mesh.hop", "Mesh Hop Count", base.DEC, nil, 0xF0)
local f_mesh_ttl  = ProtoField.uint8("opennear.mesh.ttl", "Mesh TTL", base.DEC, nil, 0x0F)
local f_mesh_src  = ProtoField.uint16("opennear.mesh.src_id", "Mesh Origin Src-ID", base.HEX)
local f_mesh_dst  = ProtoField.uint16("opennear.mesh.dst_id", "Mesh Target Dst-ID", base.HEX)
local f_mesh_mid  = ProtoField.uint16("opennear.mesh.msg_id", "Mesh Msg ID", base.DEC)
local f_mesh_plen = ProtoField.uint8("opennear.mesh.plen", "Mesh Payload Len", base.DEC)
local f_mesh_mac  = ProtoField.ether("opennear.mesh.mac48", "Mesh Node MAC48")
local f_crc16     = ProtoField.uint16("opennear.crc16", "Payload CRC-16", base.HEX)

p_on.fields = {
    f_meta_ch, f_meta_node, f_ver, f_type, f_sec, f_subg, f_src, f_dst,
    f_seq_low, f_slot, f_hcrc8, f_sf_seq, f_t_tx, f_backup, f_subg_id,
    f_tdma_map, f_ch_map, f_ack_map, f_salt, f_epoch, f_bmac,
    f_gw_type, f_gw_flags, f_gw_orig, f_gw_seq, f_gw_plen, f_gw_mac,
    f_mesh_type, f_mesh_hop, f_mesh_ttl, f_mesh_src, f_mesh_dst, f_mesh_mid, f_mesh_plen, f_mesh_mac,
    f_crc16
}

function p_on.dissector(tvbuf, pinfo, tree)
    local pktlen = tvbuf:len()
    if pktlen < 12 then return end -- 4B meta + 6B hdr + 2B crc16

    pinfo.cols.protocol = "OpenNear"
    local root = tree:add(p_on, tvbuf(), "OpenNear Deterministic Wireless Frame")

    -- 4-byte PCAP radio metadata header
    local meta = root:add(tvbuf(0, 4), "Radio Tap Metadata")
    meta:add(f_meta_ch, tvbuf(0, 1))
    meta:add(f_meta_node, tvbuf(1, 1))

    -- 6-byte Compact MAC Header
    local hdr = root:add(tvbuf(4, 6), "OpenNear 6-Byte Compact MAC Header")
    local fc = tvbuf(4, 1):uint()
    local ftype = bit.band(bit.rshift(fc, 2), 0x0F)
    hdr:add(f_ver, tvbuf(4, 1))
    hdr:add(f_type, tvbuf(4, 1))
    hdr:add(f_sec, tvbuf(4, 1))
    hdr:add(f_subg, tvbuf(4, 1))

    local b1 = tvbuf(5, 1):uint()
    local b2 = tvbuf(6, 1):uint()
    local b3 = tvbuf(7, 1):uint()
    local src_id = bit.bor(bit.lshift(b1, 4), bit.rshift(b2, 4))
    local dst_id = bit.bor(bit.lshift(bit.band(b2, 0x0F), 8), b3)
    hdr:add(f_src, tvbuf(5, 2), src_id)
    hdr:add(f_dst, tvbuf(6, 2), dst_id)
    hdr:add(f_seq_low, tvbuf(8, 1))
    hdr:add(f_slot, tvbuf(8, 1))
    hdr:add(f_hcrc8, tvbuf(9, 1))

    local names = { [0]="SYNC_MAP", [1]="SUB_SYNC", [2]="TDMA_DATA", [3]="JOIN_REQ",
                    [4]="JOIN_ACK", [5]="RTT_REQ", [6]="RTT_RESP", [7]="CRA_ALERT",
                    [8]="OTA_META", [9]="OTA_CHUNK", [10]="OTA_STATUS", [11]="OTA_COMMIT",
                    [12]="GW_BRIDGE", [13]="MESH" }
    pinfo.cols.info = string.format("%s 0x%03X -> 0x%03X (CH=%d)",
                                    names[ftype] or "UNKNOWN", src_id, dst_id, tvbuf(0,1):uint())

    -- Decode SYNC_MAP / SUB_SYNC payload (44 bytes)
    if (ftype == 0 or ftype == 1) and pktlen >= 4 + 6 + 44 + 2 then
        local pt = root:add(tvbuf(10, 44), "SYNC / MAP Schedule Payload")
        pt:add_le(f_sf_seq,   tvbuf(10, 4))
        pt:add_le(f_t_tx,     tvbuf(14, 8))
        pt:add_le(f_backup,   tvbuf(24, 2))
        pt:add_le(f_subg_id,  tvbuf(26, 2))
        pt:add_le(f_tdma_map, tvbuf(32, 2))
        pt:add_le(f_ch_map,   tvbuf(36, 2))
        pt:add_le(f_ack_map,  tvbuf(38, 2))
        pt:add_le(f_salt,     tvbuf(40, 4))
        pt:add(f_epoch,       tvbuf(44, 1))
        pt:add(f_bmac,        tvbuf(50, 4))
    elseif ftype == 12 and bit.band(fc, 0x02) == 0 and pktlen >= 4 + 6 + 7 + 2 then
        -- Unencrypted GW_BRIDGE payload (7B header + payload)
        local gwt = tvbuf(10, 1):uint()
        local plen = tvbuf(16, 1):uint()
        local pt = root:add(tvbuf(10, pktlen - 12), "GW_BRIDGE Payload")
        pt:add(f_gw_type,    tvbuf(10, 1))
        pt:add(f_gw_flags,   tvbuf(11, 1))
        pt:add_le(f_gw_orig, tvbuf(12, 2))
        pt:add_le(f_gw_seq,  tvbuf(14, 2))
        pt:add(f_gw_plen,    tvbuf(16, 1))
        if gwt == 0x09 and plen >= 6 and pktlen >= 4 + 6 + 13 + 2 then
            pt:add(f_gw_mac, tvbuf(17, 6))
        end
    elseif ftype == 13 and bit.band(fc, 0x02) == 0 and pktlen >= 4 + 6 + 9 + 2 then
        -- Unencrypted 3~6-hop Hybrid MESH payload (9B header + payload)
        local mt = tvbuf(10, 1):uint()
        local plen = tvbuf(18, 1):uint()
        local pt = root:add(tvbuf(10, pktlen - 12), "Hybrid Mesh Routing Payload")
        pt:add(f_mesh_type,    tvbuf(10, 1))
        pt:add(f_mesh_hop,     tvbuf(11, 1))
        pt:add(f_mesh_ttl,     tvbuf(11, 1))
        pt:add_le(f_mesh_src,  tvbuf(12, 2))
        pt:add_le(f_mesh_dst,  tvbuf(14, 2))
        pt:add_le(f_mesh_mid,  tvbuf(16, 2))
        pt:add(f_mesh_plen,    tvbuf(18, 1))
        if mt == 0x04 and plen >= 6 and pktlen >= 4 + 6 + 15 + 2 then
            pt:add(f_mesh_mac, tvbuf(19, 6))
        end
    end

    root:add_le(f_crc16, tvbuf(pktlen - 2, 2))
end

local wtap_encap = DissectorTable.get("wtap_encap")
wtap_encap:add(wtap.USER0, p_on)
