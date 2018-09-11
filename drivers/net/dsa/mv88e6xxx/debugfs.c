#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

/* mv88e6xxx module debugfs directory */
static struct dentry *mv88e6xxx_dbg_dir;

struct mv88e6xxx_dbg_ops {
	int (*read)(struct mv88e6xxx_chip *chip, int id, struct seq_file *seq);
	int (*write)(struct mv88e6xxx_chip *chip, int id, char *buf);
};

struct mv88e6xxx_dbg_priv {
	const struct mv88e6xxx_dbg_ops *ops;
	struct mv88e6xxx_chip *chip;
	int id;
};

static int mv88e6xxx_dbg_show(struct seq_file *seq, void *p)
{
	struct mv88e6xxx_dbg_priv *priv = seq->private;
	struct mv88e6xxx_chip *chip = priv->chip;
	int err;

	if (!priv->ops->read) {
		pr_err("file mode bypassed???\n");
		return -EOPNOTSUPP;
	}

	mutex_lock(&chip->reg_lock);
	err = priv->ops->read(chip, priv->id, seq);
	mutex_unlock(&chip->reg_lock);

	return err;
}

static ssize_t mv88e6xxx_dbg_write(struct file *file,
				   const char __user *user_buf, size_t count,
				   loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct mv88e6xxx_dbg_priv *priv = seq->private;
	struct mv88e6xxx_chip *chip = priv->chip;
	char buf[count + 1];
	int err;

	if (!priv->ops->write) {
		pr_err("file mode bypassed???\n");
		return -EOPNOTSUPP;
	}

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = '\0';

	mutex_lock(&chip->reg_lock);
	err = priv->ops->write(chip, priv->id, buf);
	mutex_unlock(&chip->reg_lock);

	return err ? err : count;
}

static int mv88e6xxx_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, mv88e6xxx_dbg_show, inode->i_private);
}

static const struct file_operations mv88e6xxx_dbg_fops = {
	.open = mv88e6xxx_dbg_open,
	.read = seq_read,
	.write = mv88e6xxx_dbg_write,
	.llseek = no_llseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int mv88e6xxx_dbg_create_file(struct mv88e6xxx_chip *chip,
				     struct dentry *dir, char *name, int id,
				     const struct mv88e6xxx_dbg_ops *ops)
{
	struct mv88e6xxx_dbg_priv *priv;
	struct dentry *entry;
	umode_t mode;

	priv = devm_kzalloc(chip->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->chip = chip;
	priv->ops = ops;
	priv->id = id;

	mode = 0;
	if (ops->read)
		mode |= S_IRUGO;
	if (ops->write)
		mode |= S_IWUSR;

	entry = debugfs_create_file(name, mode, dir, priv, &mv88e6xxx_dbg_fops);
	if (!entry)
		return -EFAULT;

	if (IS_ERR(entry))
		return PTR_ERR(entry);

	return 0;
}

static const char * const mv88e6xxx_port_8021q_mode_names[] = {
	[MV88E6XXX_PORT_CTL2_8021Q_MODE_DISABLED] = "Disabled",
	[MV88E6XXX_PORT_CTL2_8021Q_MODE_FALLBACK] = "Fallback",
	[MV88E6XXX_PORT_CTL2_8021Q_MODE_CHECK] = "Check",
	[MV88E6XXX_PORT_CTL2_8021Q_MODE_SECURE] = "Secure",
};

static int mv88e6xxx_dbg_8021q_mode_read(struct mv88e6xxx_chip *chip, int id,
					 struct seq_file *seq)
{
	u16 val;
	int err;

	err = mv88e6xxx_port_read(chip, id, MV88E6XXX_PORT_CTL2, &val);
	if (err)
		return err;

	val &= MV88E6XXX_PORT_CTL2_8021Q_MODE_MASK;

	seq_printf(seq, " %s\n", mv88e6xxx_port_8021q_mode_names[val]);

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_8021q_mode_ops = {
	.read = mv88e6xxx_dbg_8021q_mode_read,
};

static int mv88e6xxx_g1_atu_get_age_time(struct mv88e6xxx_chip *chip,
					 unsigned int *msecs)
{
	u8 age_time;
	u16 val;
	int err;

	err = mv88e6xxx_g1_read(chip, MV88E6XXX_G1_ATU_CTL, &val);
	if (err)
		return err;

	/* AgeTime is 11:4 bits */
	age_time = (val & 0xff0) >> 4;
	*msecs = age_time * chip->info->age_time_coeff;

	return 0;
}

static int mv88e6xxx_dbg_age_time_read(struct mv88e6xxx_chip *chip, int id,
				       struct seq_file *seq)
{
	unsigned int msecs;
	int err;

	err = mv88e6xxx_g1_atu_get_age_time(chip, &msecs);
	if (err)
		return err;

	seq_printf(seq, "%d\n", msecs);

	return 0;
}

static int mv88e6xxx_dbg_age_time_write(struct mv88e6xxx_chip *chip, int id,
					char *buf)
{
	unsigned int msecs;

	if (kstrtouint(buf, 10, &msecs))
		return -EINVAL;

	return mv88e6xxx_g1_atu_set_age_time(chip, msecs);
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_age_time_ops = {
	.read = mv88e6xxx_dbg_age_time_read,
	.write = mv88e6xxx_dbg_age_time_write,
};

static const char * const mv88e6xxx_atu_unicast_state_names[] = {
	[0x0] = "UNUSED", /* MV88E6XXX_G1_ATU_DATA_STATE_UNUSED */
	[0x1] = "Age 1 (oldest)",
	[0x2] = "Age 2",
	[0x3] = "Age 3",
	[0x4] = "Age 4",
	[0x5] = "Age 5",
	[0x6] = "Age 6",
	[0x7] = "Age 7 (newest)",
	[0x8] = "UC_STATIC_POLICY",
	[0x9] = "UC_STATIC_POLICY_PO",
	[0xa] = "UC_STATIC_NRL",
	[0xb] = "UC_STATIC_NRL_PO",
	[0xc] = "UC_STATIC_MGMT",
	[0xd] = "UC_STATIC_MGMT_PO", /* MV88E6XXX_G1_ATU_DATA_STATE_UC_MGMT */
	[0xe] = "UC_STATIC", /* MV88E6XXX_G1_ATU_DATA_STATE_UC_STATIC */
	[0xf] = "UC_STATIC_PO", /* MV88E6XXX_G1_ATU_DATA_STATE_UC_PRIO_OVER */
};

static const char * const mv88e6xxx_atu_multicast_state_names[] = {
	[0x0] = "UNUSED", /* MV88E6XXX_G1_ATU_DATA_STATE_UNUSED */
	[0x1] = "RESERVED",
	[0x2] = "RESERVED",
	[0x3] = "RESERVED",
	[0x4] = "MC_STATIC_POLICY",
	[0x5] = "MC_STATIC_NRL", /* MV88E6XXX_G1_ATU_DATA_STATE_MC_NONE_RATE */
	[0x6] = "MC_STATIC_MGMT",
	[0x7] = "MC_STATIC", /* MV88E6XXX_G1_ATU_DATA_STATE_MC_STATIC */
	[0x8] = "RESERVED",
	[0x9] = "RESERVED",
	[0xa] = "RESERVED",
	[0xb] = "RESERVED",
	[0xc] = "MC_STATIC_POLICY_PO",
	[0xd] = "MC_STATIC_NRL_PO",
	[0xe] = "MC_STATIC_MGMT_PO", /* MV88E6XXX_G1_ATU_DATA_STATE_MC_MGMT */
	[0xf] = "MC_STATIC_PO", /* MV88E6XXX_G1_ATU_DATA_STATE_MC_PRIO_OVER */
};

static void mv88e6xxx_dbg_atu_read_ent(struct mv88e6xxx_chip *chip,
				       struct seq_file *seq, u16 fid,
				       const struct mv88e6xxx_atu_entry *entry)
{
	int i;

	/* FID */
	seq_printf(seq, "%4d", fid);

	/* MAC address */
	seq_printf(seq, "  %.2x", entry->mac[0]);
	for (i = 1; i < ETH_ALEN; ++i)
		seq_printf(seq, ":%.2x", entry->mac[i]);

	/* State */
	seq_printf(seq, "  %19s", is_multicast_ether_addr(entry->mac) ?
		   mv88e6xxx_atu_multicast_state_names[entry->state] :
		   mv88e6xxx_atu_unicast_state_names[entry->state]);

	/* Trunk ID or Port vector */
	if (entry->trunk) {
		seq_printf(seq, "       y  %d", entry->portvec);
	} else {
		seq_puts(seq, "       n ");
		for (i = 0; i < mv88e6xxx_num_ports(chip); ++i)
			if (entry->portvec & BIT(i))
				seq_printf(seq, " %d", i);
			else
				seq_puts(seq, " -");
	}

	seq_puts(seq, "\n");
}

static int mv88e6xxx_dbg_atu_read(struct mv88e6xxx_chip *chip, int id,
				  struct seq_file *seq)
{
	struct mv88e6xxx_atu_entry next;
	int err;

	seq_puts(seq, " FID  MAC Addr                  State         Trunk?  DPV/Trunk ID\n");

	next.state = MV88E6XXX_G1_ATU_DATA_STATE_UNUSED;
	eth_broadcast_addr(next.mac);

	do {
		err = mv88e6xxx_g1_atu_getnext(chip, id, &next);
		if (err)
			return err;

		if (next.state == MV88E6XXX_G1_ATU_DATA_STATE_UNUSED)
			break;

		mv88e6xxx_dbg_atu_read_ent(chip, seq, id, &next);
	} while (!is_broadcast_ether_addr(next.mac));

	return 0;
}

static int mv88e6xxx_dbg_atu_write(struct mv88e6xxx_chip *chip, int id,
				   char *buf)
{
	struct mv88e6xxx_atu_entry entry = { 0 };
	unsigned int port;
	int ret;

	ret = sscanf(buf, "%u %2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx", &port,
		     &entry.mac[0], &entry.mac[1], &entry.mac[2],
		     &entry.mac[3], &entry.mac[4], &entry.mac[5]);
	if (ret == 0)
		return mv88e6xxx_g1_atu_flush(chip, id, true);

	if (port >= mv88e6xxx_num_ports(chip))
		return -ERANGE;

	if (ret == 1)
		return mv88e6xxx_g1_atu_remove(chip, id, port, true);

	if (ret != 7)
		return -EINVAL;

	entry.state = 4;
	entry.trunk = false;
	entry.portvec = BIT(port);

	return mv88e6xxx_g1_atu_loadpurge(chip, id, &entry);
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_atu_ops = {
		.read = mv88e6xxx_dbg_atu_read,
		.write = mv88e6xxx_dbg_atu_write,
};

static int mv88e6xxx_dbg_default_vid_read(struct mv88e6xxx_chip *chip, int id,
					  struct seq_file *seq)
{
	u16 pvid;
	int err;

	err = mv88e6xxx_port_get_pvid(chip, id, &pvid);
	if (err)
		return err;

	seq_printf(seq, "%d\n", pvid);

	return 0;
}

static int mv88e6xxx_dbg_default_vid_write(struct mv88e6xxx_chip *chip, int id,
					   char *buf)
{
	u16 pvid;

	if (kstrtou16(buf, 10, &pvid))
		return -EINVAL;

	if (pvid >= VLAN_N_VID)
		return -ERANGE;

	return mv88e6xxx_port_set_pvid(chip, id, pvid);
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_default_vid_ops = {
	.read = mv88e6xxx_dbg_default_vid_read,
	.write = mv88e6xxx_dbg_default_vid_write,
};

// TODO write mv88e6xxx_g2_device_mapping_read()
static int mv88e6xxx_dbg_device_map_read(struct mv88e6xxx_chip *chip, int id,
					 struct seq_file *seq)
{
	u16 val;
	int target, err;

	seq_puts(seq, "Target Port\n");

	for (target = 0; target < 32; ++target) {
		val = target << __bf_shf(MV88E6XXX_G2_DEVICE_MAPPING_DEV_MASK);
		err = mv88e6xxx_write(chip, chip->info->global2_addr,
				      MV88E6XXX_G2_DEVICE_MAPPING, val);
		if (err)
			return err;

		err = mv88e6xxx_read(chip, chip->info->global2_addr,
				     MV88E6XXX_G2_DEVICE_MAPPING, &val);
		if (err)
			return err;

		seq_printf(seq, "  %2d   %2d\n", target,
			   val & MV88E6XXX_G2_DEVICE_MAPPING_PORT_MASK);
	}

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_device_map_ops = {
	.read = mv88e6xxx_dbg_device_map_read,
};

static int mv88e6xxx_dbg_fid_read(struct mv88e6xxx_chip *chip, int id,
				  struct seq_file *seq)
{
	u16 fid;
	int err;

	err = mv88e6xxx_port_get_fid(chip, id, &fid);
	if (err)
		return err;

	seq_printf(seq, "%d\n", fid);

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_fid_ops = {
	.read = mv88e6xxx_dbg_fid_read,
};

static int mv88e6xxx_dbg_name_read(struct mv88e6xxx_chip *chip, int id,
				   struct seq_file *seq)
{
	seq_printf(seq, "%s\n", chip->info->name);

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_name_ops = {
	.read = mv88e6xxx_dbg_name_read,
};

int mv88e6xxx_g2_pvt_read(struct mv88e6xxx_chip *chip, int src_dev,
			  int src_port, u16 *data);

static int mv88e6xxx_dbg_pvt_read(struct mv88e6xxx_chip *chip, int id,
				  struct seq_file *seq)
{
	struct dsa_switch *ds;
	int src_dev, src_port;
	u16 pvlan;
	int port;
	int err;

	/* header */
	seq_puts(seq, " Dev Port PVLAN");
	for (port = 0; port < mv88e6xxx_num_ports(chip); ++port)
		seq_printf(seq, " %2d", port);
	seq_puts(seq, "\n");

	/* One line per external port */
	for (src_dev = 0; src_dev < 32; ++src_dev) {
		if (src_dev >= DSA_MAX_SWITCHES)
			break;

		ds = chip->ds->dst->ds[src_dev];
		if (!ds)
			break;

		seq_puts(seq, "\n");
		for (src_port = 0; src_port < ds->num_ports; ++src_port) {
			err = mv88e6xxx_g2_pvt_read(chip, src_dev, src_port,
						    &pvlan);
			if (err)
				return err;

			seq_printf(seq, "  %d   %2d   %03x ", src_dev,
				   src_port, pvlan);

			/* One column per internal output port */
			for (port = 0; port < mv88e6xxx_num_ports(chip); ++port)
				if (pvlan & BIT(port))
					seq_puts(seq, "  *");
				else
					seq_puts(seq, "  -");

			seq_puts(seq, "\n");
		}
	}

	return 0;
}

static int mv88e6xxx_dbg_pvt_write(struct mv88e6xxx_chip *chip, int id,
				   char *buf)
{
	const u16 mask = mv88e6xxx_port_mask(chip);
	unsigned int src_dev, src_port, pvlan;

	if (sscanf(buf, "%d %d %x", &src_dev, &src_port, &pvlan) != 3)
		return -EINVAL;

	if (src_dev >= 32 || src_port >= 16 || pvlan & ~mask)
		return -ERANGE;

	return mv88e6xxx_g2_pvt_write(chip, src_dev, src_port, pvlan);
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_pvt_ops = {
	.read = mv88e6xxx_dbg_pvt_read,
	.write = mv88e6xxx_dbg_pvt_write,
};

enum {
	/* Port Registers at 0 ... DSA_MAX_PORTS - 1 */
	MV88E6XXX_DBG_REGS_ID_GLOBAL1 = DSA_MAX_PORTS,
	MV88E6XXX_DBG_REGS_ID_GLOBAL2,
	MV88E6XXX_DBG_REGS_ID_SERDES,
};

static int mv88e6xxx_serdes_read(struct mv88e6xxx_chip *chip, int reg,
				 u16 *val)
{
	return mv88e6xxx_phy_page_read(chip, MV88E6352_ADDR_SERDES,
				       MV88E6352_SERDES_PAGE_FIBER,
				       reg, val);
}

static int mv88e6xxx_serdes_write(struct mv88e6xxx_chip *chip, int reg,
				  u16 val)
{
	return mv88e6xxx_phy_page_write(chip, MV88E6352_ADDR_SERDES,
					MV88E6352_SERDES_PAGE_FIBER,
					reg, val);
}

static int mv88e6xxx_dbg_regs_read(struct mv88e6xxx_chip *chip, int id,
				   struct seq_file *seq)
{
	u16 val;
	int reg;
	int err;

	/* Label */
	if (id == MV88E6XXX_DBG_REGS_ID_SERDES)
		seq_printf(seq, "SerDes@%d\n", chip->ds->index);
	else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL2)
		seq_printf(seq, "Global2@%d\n", chip->ds->index);
	else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL1)
		seq_printf(seq, "Global1@%d\n", chip->ds->index);
	else
		seq_printf(seq, "Port %d.%d\n", chip->ds->index, id);

	for (reg = 0; reg < 32; ++reg) {
		if (id == MV88E6XXX_DBG_REGS_ID_SERDES)
			err = mv88e6xxx_serdes_read(chip, reg, &val);
		else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL2)
			err = mv88e6xxx_read(chip, chip->info->global2_addr,
					     reg, &val);
		else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL1)
			err = mv88e6xxx_g1_read(chip, reg, &val);
		else
			err = mv88e6xxx_port_read(chip, id, reg, &val);
		if (err)
			break;

		seq_printf(seq, "%2d: %4x\n", reg, val);
	}

	return err;
}

static int mv88e6xxx_dbg_regs_write(struct mv88e6xxx_chip *chip, int id,
				    char *buf)
{
	unsigned int reg, val;
	int err;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	if (reg > 0x1f || val > 0xffff)
		return -ERANGE;

	if (id == MV88E6XXX_DBG_REGS_ID_SERDES)
		err = mv88e6xxx_serdes_write(chip, reg, val);
	else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL2)
		err = mv88e6xxx_write(chip, chip->info->global2_addr, reg, val);
	else if (id == MV88E6XXX_DBG_REGS_ID_GLOBAL1)
		err = mv88e6xxx_g1_write(chip, reg, val);
	else
		err = mv88e6xxx_port_write(chip, id, reg, val);

	return err;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_regs_ops = {
	.read = mv88e6xxx_dbg_regs_read,
	.write = mv88e6xxx_dbg_regs_write,
};

static int mv88e6xxx_scratch_wait(struct mv88e6xxx_chip *chip)
{
	return mv88e6xxx_wait(chip, chip->info->global2_addr,
			      MV88E6XXX_G2_SCRATCH_MISC_MISC,
			      MV88E6XXX_G2_SCRATCH_MISC_UPDATE);
}

static int mv88e6xxx_dbg_scratch_read(struct mv88e6xxx_chip *chip, int id,
				      struct seq_file *seq)
{
	u16 val;
	int reg, err;

	seq_puts(seq, "Register Value\n");

	for (reg = 0; reg < 0x80; ++reg) {
		val = reg << __bf_shf(MV88E6XXX_G2_SCRATCH_MISC_PTR_MASK);
		err = mv88e6xxx_write(chip, chip->info->global2_addr,
				      MV88E6XXX_G2_SCRATCH_MISC_MISC, val);
		if (err)
			return err;

		err = mv88e6xxx_scratch_wait(chip);
		if (err)
			return err;

		err = mv88e6xxx_read(chip, chip->info->global2_addr,
				     MV88E6XXX_G2_SCRATCH_MISC_MISC, &val);
		if (err)
			return err;

		seq_printf(seq, "  %2x   %2x\n", reg,
			   val & MV88E6XXX_G2_SCRATCH_MISC_DATA_MASK);
	}

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_scratch_ops = {
	.read = mv88e6xxx_dbg_scratch_read,
};

static const char * const mv88e6xxx_port_state_names[] = {
	[MV88E6XXX_PORT_CTL0_STATE_DISABLED] = "Disabled",
	[MV88E6XXX_PORT_CTL0_STATE_BLOCKING] = "Blocking", /* /Listening */
	[MV88E6XXX_PORT_CTL0_STATE_LEARNING] = "Learning",
	[MV88E6XXX_PORT_CTL0_STATE_FORWARDING] = "Forwarding",
};

static int mv88e6xxx_port_get_state(struct mv88e6xxx_chip *chip, int port,
				    u8 *state)
{
	u16 reg;
	int err;

	err = mv88e6xxx_port_read(chip, port, MV88E6XXX_PORT_CTL0, &reg);
	if (err)
		return err;

	*state = reg & MV88E6XXX_PORT_CTL0_STATE_MASK;

	return 0;
}

static int mv88e6xxx_dbg_state_read(struct mv88e6xxx_chip *chip, int id,
				    struct seq_file *seq)
{
	u8 state;
	int err;

	err = mv88e6xxx_port_get_state(chip, id, &state);
	if (err)
		return err;

	seq_printf(seq, " %s\n", mv88e6xxx_port_state_names[state]);

	return 0;
}

static int mv88e6xxx_dbg_state_write(struct mv88e6xxx_chip *chip, int id,
				     char *buf)
{
	int state;

	for (state = 0; state < ARRAY_SIZE(mv88e6xxx_port_state_names); ++state)
		if (strncasecmp(mv88e6xxx_port_state_names[state], buf,
				strlen(mv88e6xxx_port_state_names[state])) == 0)
			return mv88e6xxx_port_set_state(chip, id, state);

	return -EINVAL;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_state_ops = {
	.read = mv88e6xxx_dbg_state_read,
	.write = mv88e6xxx_dbg_state_write,
};

static int mv88e6xxx_dbg_stats_read(struct mv88e6xxx_chip *chip, int id,
				    struct seq_file *seq)
{
	int port = id;
	int stat;
	int err;
	int types = 0;
	u16 bank1_select;
	u16 histogram;
	u64 value;

	if (chip->info->ops->stats_get_stats == mv88e6095_stats_get_stats) {
		types = STATS_TYPE_BANK0 | STATS_TYPE_PORT;
		bank1_select = 0;
		histogram = MV88E6XXX_G1_STATS_OP_HIST_RX_TX;
	}

	if (chip->info->ops->stats_get_stats == mv88e6320_stats_get_stats) {
		types = STATS_TYPE_BANK0 | STATS_TYPE_BANK1;
		bank1_select = MV88E6XXX_G1_STATS_OP_BANK_1_BIT_9;
		histogram = MV88E6XXX_G1_STATS_OP_HIST_RX_TX;
	}

	if (chip->info->ops->stats_get_stats == mv88e6390_stats_get_stats) {
		types = STATS_TYPE_BANK0 | STATS_TYPE_BANK1;
		bank1_select = MV88E6XXX_G1_STATS_OP_BANK_1_BIT_10;
		histogram = 0;
	}

	seq_printf(seq, "         Stat       Port %d.%d\n", chip->ds->index,
		   port);

	for (stat = 0; stat < ARRAY_SIZE(mv88e6xxx_hw_stats); ++stat) {
		struct mv88e6xxx_hw_stat *hw_stat = &mv88e6xxx_hw_stats[stat];

		if (!(hw_stat->type & types))
			continue;

		seq_printf(seq, "%19s: ", hw_stat->string);

		err = mv88e6xxx_stats_snapshot(chip, port);
		if (err)
			return err;

		value = _mv88e6xxx_get_ethtool_stat(chip, hw_stat, port,
						    bank1_select, histogram);
		seq_printf(seq, "%8llu \n", value);
	}

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_stats_ops = {
	.read = mv88e6xxx_dbg_stats_read,
};

static char *mv88e6xxx_dbg_tcam_frame_type_str(int frame_type)
{
	switch (frame_type) {
	case GLOBAL3_P0_KEY1_FRAME_TYPE_NORNAL:
		return "Frame type: Nornal";
	case GLOBAL3_P0_KEY1_FRAME_TYPE_DSA:
		return "frame type: DSA";
	case GLOBAL3_P0_KEY1_FRAME_TYPE_PROVIDER:
		return "frame type: Provider";
	default:
		return "frame type: Unknown";
	}
}

static int mv88e6xxx_dbg_tcam_read_entry(struct mv88e6xxx_chip *chip,
					 struct seq_file *s, int entry,
					 struct mv88e6xxx_tcam_data *data)
{
	int err, i, value;
	u8 octet, mask;

	seq_puts(s, "      Dst          Src          Tag      Type Data\n");
	seq_printf(s, "Entry %3d\n", entry);
	seq_puts(s, "Octet:");
	for (i = 0; i < 48; i++) {
		/* -Dst-------Src-------Tag--------Eth Type----Data-- */
		if (i == 6 || i == 12 || i == 16 || i == 18 || i == 26 ||
		    i == 34 || i == 42)
			seq_puts(s, " ");

		err = mv88e6xxx_g3_tcam_get_match(chip, data, i, &octet, &mask);
		if (err)
			return err;
		seq_printf(s, "%02x", octet);
	}
	seq_puts(s, "\n");

	seq_puts(s, "Mask: ");
	for (i = 0; i < 48; i++) {
		/* -Dst-------Src-------Tag--------Eth Type----Data-- */
		if (i == 6 || i == 12 || i == 16 || i == 18 || i == 26 ||
		    i == 34 || i == 42)
			seq_puts(s, " ");

		err = mv88e6xxx_g3_tcam_get_match(chip, data, i, &octet, &mask);
		if (err)
			return err;
		seq_printf(s, "%02x", mask);
	}
	seq_puts(s, "\n");

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P0_KEY1_FRAME_TYPE,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "%s ", mv88e6xxx_dbg_tcam_frame_type_str(value));

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Source port vector: %x ", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P0_KEY3_PPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Provider priority: %d ", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P0_KEY4_PVID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Provider VLAN ID: %d ", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_INTERRUPT,
			   &value);
	seq_printf(s, "Interrupt: %d ",
		   value == GLOBAL3_P2_ACTION1_INTERRUPT);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_INC_TCAM_COUNTER,
			   &value);
	seq_printf(s, "Inc TCAM counter: %d ",
		   value == GLOBAL3_P2_ACTION1_INC_TCAM_COUNTER);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION1_VID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "VID: %d ", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_FLOW_ID, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Flow ID: %d ",
			   value - GLOBAL3_P2_ACTION2_FLOW_ID_0);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_QPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Queue priority: %d ",
			   value - GLOBAL3_P2_ACTION2_QPRI_0);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION2_FPRI, &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Priority: %d ",
			   value - GLOBAL3_P2_ACTION2_FPRI_0);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Destination port vector: %x ", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION4_FRAME_ACTION,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED) {
		seq_printf(s, "Frame Action: %x ", value);
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SRC_IS_TAGGED)
			seq_puts(s, "SRC_IS_TAGGED ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_PVID)
			seq_puts(s, "PVID ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_MGMT)
			seq_puts(s, "MGMT ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SNOOP)
			seq_puts(s, "SNOOP ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_MIRROR)
			seq_puts(s, "POLICY_MIRROR ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_POLICY_TRAP)
			seq_puts(s, "POLICY_TRAP ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_SANRL)
			seq_puts(s, "SaNRL ");
		if (value & GLOBAL3_P2_ACTION4_FRAME_ACTION_DANRL)
			seq_puts(s, "DaNRL ");
	}

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_ACTION4_LOAD_BALANCE,
			   &value);
	if (value != MV88E6XXX_TCAM_PARAM_DISABLED)
		seq_printf(s, "Load balance: %d", value);

	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_DEBUG_PORT, &value);
	seq_printf(s, "Debug Port: %d ", value);
	mv88e6xxx_g3_tcam_get(chip, data, MV88E6XXX_P2_DEBUG_HIT, &value);
	seq_printf(s, "Debug Hit %x\n", value);

	return 0;
}

static int mv88e6xxx_dbg_tcam_read(struct mv88e6xxx_chip *chip, int id,
				   struct seq_file *seq)
{
	struct mv88e6xxx_tcam_data data;
	int err;

	err = mv88e6xxx_g3_tcam_read(chip, id, &data);
	if (err)
		return err;

	return mv88e6xxx_dbg_tcam_read_entry(chip, seq, id, &data);
}

static int mv88e6xxx_dbg_tcam_write(struct mv88e6xxx_chip *chip, int id,
				    char *buf)
{
	struct mv88e6xxx_tcam_data data;

	memset(&data, 0, sizeof(data));

	mv88e6xxx_g3_tcam_flush_all(chip);
	mv88e6xxx_port_enable_tcam(chip, 0);
	mv88e6xxx_port_enable_tcam(chip, 1);

	/* Destination - Broadcast address */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 0, 0xff, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 1, 0xff, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 2, 0xff, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 3, 0xff, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 4, 0xff, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 5, 0xff, 0xff);

	/* Source Port 0 */
	mv88e6xxx_g3_tcam_set(chip, &data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   (1 << 0));

	/* Destination port None, i.e. drop */
	mv88e6xxx_g3_tcam_set(chip, &data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   0);

	mv88e6xxx_g3_tcam_load_entry(chip, 42, &data);

	memset(&data, 0, sizeof(data));

	/* Source 00:26:55:d2:27:a9 */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 6, 0x00, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 7, 0x26, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 8, 0x55, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 9, 0xd2, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 10, 0x27, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 11, 0xa9, 0xff);

	/* Ether Type 0x0806 - ARP */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 16, 0x08, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 17, 0x06, 0xff);

	/* ARP Hardware Type 1  - Ethernet */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 18, 0x00, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 19, 0x01, 0xff);

	/* ARP protocol Type 0x0800 - IP */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 20, 0x08, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 21, 0x00, 0xff);

	/* Operation 2 - reply */
	mv88e6xxx_g3_tcam_set_match(chip, &data, 24, 0x00, 0xff);
	mv88e6xxx_g3_tcam_set_match(chip, &data, 25, 0x02, 0xff);

	/* Source Port 1 */
	mv88e6xxx_g3_tcam_set(chip, &data, MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
			   (1 << 1));

	/* Destination port None, i.e. drop */
	mv88e6xxx_g3_tcam_set(chip, &data, MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
			   0);

	mv88e6xxx_g3_tcam_load_entry(chip, 43, &data);

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_tcam_ops = {
	.read = mv88e6xxx_dbg_tcam_read,
	.write = mv88e6xxx_dbg_tcam_write,
};

static int mv88e6xxx_dbg_tcam_dump_read(struct mv88e6xxx_chip *chip, int id,
					   struct seq_file *seq)
{
	struct mv88e6xxx_tcam_data data;
	int entry = 0;
	int err;

	while (1) {
		err = mv88e6xxx_g3_tcam_get_next(chip, &entry, &data);
		if (err)
			return err;

		if (entry == 0xff)
			break;

		err = mv88e6xxx_dbg_tcam_read_entry(chip, seq, entry, &data);
		if (err)
			return err;
	}

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_tcam_dump_ops = {
	.read = mv88e6xxx_dbg_tcam_dump_read,
};

static int mv88e6xxx_dbg_vlan_table_read(struct mv88e6xxx_chip *chip, int id,
					 struct seq_file *seq)
{
	u16 val;
	int i, err;

	/* header */
	seq_puts(seq, " Port");
	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i)
		seq_printf(seq, " %2d", i);
	seq_puts(seq, "\n");

	seq_printf(seq, "%4d ", id);

	err = mv88e6xxx_port_read(chip, id, MV88E6XXX_PORT_BASE_VLAN, &val);
	if (err)
		return err;

	/* One column per output port */
	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i)
		seq_printf(seq, "  %c", val & BIT(i) ? '*' : '-');
	seq_puts(seq, "\n");

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_vlan_table_ops = {
	.read = mv88e6xxx_dbg_vlan_table_read,
};

static void mv88e6xxx_dbg_vtu_read_ent(struct mv88e6xxx_chip *chip,
				       struct seq_file *seq,
				       const struct mv88e6xxx_vtu_entry *entry)
{
	int i;

	seq_puts(seq, " VID  FID  SID");
	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i)
		seq_printf(seq, " %2d", i);
	seq_puts(seq, "\n");

	seq_printf(seq, "%4d %4d   %2d", entry->vid, entry->fid, entry->sid);

	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i) {
		switch (entry->member[i]) {
		case MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_UNMODIFIED:
			seq_puts(seq, "  =");
			break;
		case MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_UNTAGGED:
			seq_puts(seq, "  u");
			break;
		case MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_TAGGED:
			seq_puts(seq, "  t");
			break;
		case MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_NON_MEMBER:
			seq_puts(seq, "  x");
			break;
		default:
			seq_puts(seq, " ??");
			break;
		}
	}

	seq_puts(seq, "\n");
}

#ifdef VTU_PER_VID_ENTRY
static int mv88e6xxx_dbg_vtu_read(struct mv88e6xxx_chip *chip, int id,
				  struct seq_file *seq)
{
	struct mv88e6xxx_vtu_entry entry;
	int err;

	err = mv88e6xxx_vtu_get(chip, id, &entry, false);
	if (!err)
		mv88e6xxx_dbg_vtu_read_ent(chip, seq, &entry);

	return err == -EOPNOTSUPP ? 0 : err;
}

static int mv88e6xxx_dbg_vtu_write(struct mv88e6xxx_chip *chip, int id,
				   char *buf)
{
	struct mv88e6xxx_vtu_entry entry = { 0 };
	bool valid = true;
	char tags[12]; /* DSA_MAX_PORTS */
	int ret, port, vid, fid, sid;

	/* scan 12 chars instead of num_ports to avoid dynamic scanning... */
	ret = sscanf(buf, "%d %d %d %c %c %c %c %c %c %c %c %c %c %c %c", &vid,
		     &fid, &sid, &tags[0], &tags[1], &tags[2], &tags[3],
		     &tags[4], &tags[5], &tags[6], &tags[7], &tags[8], &tags[9],
		     &tags[10], &tags[11]);
	if (ret == 1)
		valid = false;
	else if (ret != 3 + mv88e6xxx_num_ports(chip))
		return -EINVAL;

	entry.vid = vid;
	entry.valid = valid;

	if (valid) {
		entry.fid = fid;
		entry.sid = sid;
		/* Note: The VTU entry pointed by VID will be loaded but not
		 * considered valid until the STU entry pointed by SID is valid.
		 */

		for (port = 0; port < mv88e6xxx_num_ports(chip); ++port) {
			u8 tag;

			switch (tags[port]) {
			case 'u':
				tag = MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_UNTAGGED;
				break;
			case 't':
				tag = MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_TAGGED;
				break;
			case 'x':
				tag = MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_NON_MEMBER;
				break;
			case '=':
				tag = MV88E6XXX_G1_VTU_DATA_MEMBER_TAG_UNMODIFIED;
				break;
			default:
				return -EINVAL;
			}

			entry.data[port] = tag;
		}
	}

	return mv88e6xxx_vtu_loadpurge(chip, &entry);
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_vtu_ops = {
	.read = mv88e6xxx_dbg_vtu_read,
	.write = mv88e6xxx_dbg_vtu_write,
};
#endif

static int mv88e6xxx_dbg_vtu_dump_read(struct mv88e6xxx_chip *chip, int id,
					  struct seq_file *seq)
{
	struct mv88e6xxx_vtu_entry next = {
		.vid = chip->info->max_vid,
	};
	int err;

	do {
		err = mv88e6xxx_vtu_getnext(chip, &next);
		if (err)
			return err;

		if (!next.valid)
			break;

		mv88e6xxx_dbg_vtu_read_ent(chip, seq, &next);
	} while (next.vid < chip->info->max_vid);

	return 0;
}

static const struct mv88e6xxx_dbg_ops mv88e6xxx_dbg_vtu_dump_ops = {
	.read = mv88e6xxx_dbg_vtu_dump_read,
};

static int mv88e6xxx_dbg_init_atu(struct mv88e6xxx_chip *chip)
{
	struct dentry *dir;
	char name[32];
	int fid;
	int err;

	dir = debugfs_create_dir("atu", chip->debugfs_dir);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir))
		return PTR_ERR(dir);

	for (fid = 0; fid < mv88e6xxx_num_databases(chip); ++fid) {
		snprintf(name, sizeof(name), "%d", fid);

		err = mv88e6xxx_dbg_create_file(chip, dir, name, fid,
						&mv88e6xxx_dbg_atu_ops);
		if (err)
			break;
	}

	return err;
}


static int mv88e6xxx_dbg_port_symlink(struct mv88e6xxx_chip *chip,
				      struct dentry *dir, int port)
{
	struct net_device *netdev;
	struct dentry *symlink;
	char name[54];

	if (dsa_is_cpu_port(chip->ds, port))
		netdev = chip->ds->dst->cpu_dp->netdev;
	else
		netdev = chip->ds->ports[port].netdev;

	if (!netdev)
		return 0;

	snprintf(name, sizeof(name), "/sys/class/net/%s", netdev_name(netdev));

	symlink = debugfs_create_symlink("net", dir, name);
	if (!symlink)
		return -EFAULT;

	if (IS_ERR(symlink))
		return PTR_ERR(symlink);

	return 0;
}

static int mv88e6xxx_dbg_init_port(struct mv88e6xxx_chip *chip, int port)
{
	struct dentry *dir;
	char name[32];
	int err;

	snprintf(name, sizeof(name), "p%d", port);

	dir = debugfs_create_dir(name, chip->debugfs_dir);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir))
		return PTR_ERR(dir);

	err = mv88e6xxx_dbg_port_symlink(chip, dir, port);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "8021q_mode", port,
					&mv88e6xxx_dbg_8021q_mode_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "default_vid", port,
					&mv88e6xxx_dbg_default_vid_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "fid", port,
					&mv88e6xxx_dbg_fid_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "regs", port,
					&mv88e6xxx_dbg_regs_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "state", port,
					&mv88e6xxx_dbg_state_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "stats", port,
					&mv88e6xxx_dbg_stats_ops);
	if (err)
		return err;

	err = mv88e6xxx_dbg_create_file(chip, dir, "vlan_table", port,
					&mv88e6xxx_dbg_vlan_table_ops);
	if (err)
		return err;

	return 0;
}

static int mv88e6xxx_dbg_init_tcam(struct mv88e6xxx_chip *chip)
{
	struct dentry *dir;
	char name[32];
	int entry;
	int err;

	dir = debugfs_create_dir("tcam", chip->debugfs_dir);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir))
		return PTR_ERR(dir);

	for (entry = 0; entry < 255; ++entry) {
		snprintf(name, sizeof(name), "%d", entry);

		err = mv88e6xxx_dbg_create_file(chip, dir, name, entry,
						&mv88e6xxx_dbg_tcam_ops);
		if (err)
			return err;
	}

	return mv88e6xxx_dbg_create_file(chip, dir, "dump", -1,
					 &mv88e6xxx_dbg_tcam_dump_ops);
}

static int mv88e6xxx_dbg_init_vtu(struct mv88e6xxx_chip *chip)
{
#ifdef VTU_PER_VID_ENTRY
	struct dentry *dir;
	char name[32];
	int vid;
	int err;

	dir = debugfs_create_dir("vtu", chip->debugfs_dir);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir))
		return PTR_ERR(dir);

	for (vid = 0; vid < 4096; ++vid) {
		snprintf(name, sizeof(name), "%d", vid);

		err = mv88e6xxx_dbg_create_file(chip, dir, name, vid,
						&mv88e6xxx_dbg_vtu_ops);
		if (err)
			return err;
	}

	return mv88e6xxx_dbg_create_file(chip, dir, "dump", -1,
					 &mv88e6xxx_dbg_vtu_dump_ops);
#else
	return mv88e6xxx_dbg_create_file(chip, chip->debugfs_dir, "vtu", -1,
					 &mv88e6xxx_dbg_vtu_dump_ops);
#endif
}

static void mv88e6xxx_dbg_destroy_chip(struct mv88e6xxx_chip *chip)
{
	/* handles NULL */
	debugfs_remove_recursive(chip->debugfs_dir);
}

static int mv88e6xxx_dbg_init_chip(struct mv88e6xxx_chip *chip)
{
	struct dentry *dir;
	char name[32];
	int err;
	int i;

	/* skip if there is no debugfs support */
	if (!mv88e6xxx_dbg_dir)
		return 0;

	snprintf(name, sizeof(name), "sw%d", chip->ds->index);

	dir = debugfs_create_dir(name, mv88e6xxx_dbg_dir);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir))
		return PTR_ERR(dir);

	chip->debugfs_dir = dir;

	err = mv88e6xxx_dbg_create_file(chip, dir, "age_time", -1,
					&mv88e6xxx_dbg_age_time_ops);
	if (err)
		goto destroy;

	err = mv88e6xxx_dbg_create_file(chip, dir, "device_map", -1,
					&mv88e6xxx_dbg_device_map_ops);
	if (err)
		goto destroy;


	err = mv88e6xxx_dbg_create_file(chip, dir, "global1",
					MV88E6XXX_DBG_REGS_ID_GLOBAL1,
					&mv88e6xxx_dbg_regs_ops);
	if (err)
		goto destroy;

	if (chip->info->global2_addr) {
		err = mv88e6xxx_dbg_create_file(chip, dir, "global2",
						MV88E6XXX_DBG_REGS_ID_GLOBAL2,
						&mv88e6xxx_dbg_regs_ops);
		if (err)
			goto destroy;
	}

	err = mv88e6xxx_dbg_create_file(chip, dir, "name", -1,
					&mv88e6xxx_dbg_name_ops);
	if (err)
		goto destroy;

	if (mv88e6xxx_has_pvt(chip)) {
		err = mv88e6xxx_dbg_create_file(chip, dir, "pvt", -1,
						&mv88e6xxx_dbg_pvt_ops);
		if (err)
			goto destroy;
	}

	err = mv88e6xxx_dbg_create_file(chip, dir, "scratch", -1,
					&mv88e6xxx_dbg_scratch_ops);
	if (err)
		goto destroy;

	if (chip->info->ops->serdes_power) {
		err = mv88e6xxx_dbg_create_file(chip, dir, "serdes",
						MV88E6XXX_DBG_REGS_ID_SERDES,
						&mv88e6xxx_dbg_regs_ops);
		if (err)
			goto destroy;
	}

	err = mv88e6xxx_dbg_init_atu(chip);
	if (err)
		goto destroy;

	for (i = 0; i < mv88e6xxx_num_ports(chip); ++i) {
		err = mv88e6xxx_dbg_init_port(chip, i);
		if (err)
			goto destroy;
	}

	if (chip->info->ops->port_enable_tcam) {
		err = mv88e6xxx_dbg_init_tcam(chip);
		if (err)
			goto destroy;
	}

	if (chip->info->max_vid) {
		err = mv88e6xxx_dbg_init_vtu(chip);
		if (err)
			goto destroy;
	}

	return 0;
destroy:
	mv88e6xxx_dbg_destroy_chip(chip);

	return err;
}

static void mv88e6xxx_dbg_destroy_module(void)
{
	debugfs_remove_recursive(mv88e6xxx_dbg_dir);
}

static int mv88e6xxx_dbg_init_module(void)
{
	struct dentry *dir;
	int err;

	dir = debugfs_create_dir("mv88e6xxx", NULL);
	if (!dir)
		return -EFAULT;

	if (IS_ERR(dir)) {
		err = PTR_ERR(dir);

		/* kernel built without debugfs support */
		if (err == -ENODEV)
			return 0;

		return err;
	}

	mv88e6xxx_dbg_dir = dir;

	return 0;
}
