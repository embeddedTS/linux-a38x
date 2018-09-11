/*
 * net/dsa/debugfs.c - DSA debugfs interface
 * Copyright (c) 2017 Savoir-faire Linux, Inc.
 *	Vivien Didelot <vivien.didelot@savoirfairelinux.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/debugfs.h>
#include <linux/etherdevice.h>
#include <linux/seq_file.h>

#include "dsa_priv.h"

#define DSA_SWITCH_FMT	"switch%d"
#define DSA_PORT_FMT	"port%d"

/* DSA module debugfs directory */
static struct dentry *dsa_debugfs_dir;

struct dsa_debugfs_ops {
	int (*read)(struct dsa_switch *ds, int id, struct seq_file *seq);
	int (*write)(struct dsa_switch *ds, int id, char *buf);
};

struct dsa_debugfs_priv {
	const struct dsa_debugfs_ops *ops;
	struct dsa_switch *ds;
	int id;
};

static int dsa_debugfs_show(struct seq_file *seq, void *p)
{
	struct dsa_debugfs_priv *priv = seq->private;
	struct dsa_switch *ds = priv->ds;

	/* Somehow file mode is bypassed... Double check here */
	if (!priv->ops->read)
		return -EOPNOTSUPP;

	return priv->ops->read(ds, priv->id, seq);
}

static ssize_t dsa_debugfs_write(struct file *file, const char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	struct dsa_debugfs_priv *priv = seq->private;
	struct dsa_switch *ds = priv->ds;
	char buf[count + 1];
	int err;

	/* Somehow file mode is bypassed... Double check here */
	if (!priv->ops->write)
		return -EOPNOTSUPP;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = '\0';

	err = priv->ops->write(ds, priv->id, buf);

	return err ? err : count;
}

static int dsa_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, dsa_debugfs_show, inode->i_private);
}

static const struct file_operations dsa_debugfs_fops = {
	.open = dsa_debugfs_open,
	.read = seq_read,
	.write = dsa_debugfs_write,
	.llseek = no_llseek,
	.release = single_release,
	.owner = THIS_MODULE,
};

static int dsa_debugfs_create_file(struct dsa_switch *ds, struct dentry *dir,
				   char *name, int id,
				   const struct dsa_debugfs_ops *ops)
{
	struct dsa_debugfs_priv *priv;
	struct dentry *entry;
	umode_t mode;

	priv = devm_kzalloc(ds->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->ops = ops;
	priv->ds = ds;
	priv->id = id;

	mode = 0;
	if (ops->read)
		mode |= 0444;
	if (ops->write)
		mode |= 0200;

	entry = debugfs_create_file(name, mode, dir, priv, &dsa_debugfs_fops);
	if (IS_ERR_OR_NULL(entry))
		return -EFAULT;

	return 0;
}

static int dsa_debugfs_fdb_dump_cb(const unsigned char *addr, u16 vid,
				   bool is_static, void *data)
{
	struct seq_file *seq = data;

	seq_printf(seq, "vid %d  %pM  %s  %s\n", vid, addr,
		   is_unicast_ether_addr(addr) ? "unicast" : "multicast",
		   is_static ? "static" : "dynamic");

	return 0;
}

static int dsa_debugfs_fdb_read(struct dsa_switch *ds, int id,
				struct seq_file *seq)
{
	if (!ds->ops->port_fdb_dump)
		return -EOPNOTSUPP;

	return ds->ops->port_fdb_dump(ds, id, dsa_debugfs_fdb_dump_cb, seq);
}

static const struct dsa_debugfs_ops dsa_debugfs_fdb_ops = {
	.read = dsa_debugfs_fdb_read,
};

static int dsa_debugfs_mdb_read(struct dsa_switch *ds, int id,
				struct seq_file *seq)
{
	if (!ds->ops->port_mdb_dump)
		return -EOPNOTSUPP;

	/* same callback as for FDB dump */
	return ds->ops->port_mdb_dump(ds, id, dsa_debugfs_fdb_dump_cb, seq);
}

static const struct dsa_debugfs_ops dsa_debugfs_mdb_ops = {
	.read = dsa_debugfs_mdb_read,
};

static void dsa_debugfs_regs_read_count(struct dsa_switch *ds, int id,
					struct seq_file *seq, int count)
{
	u16 data[count * ETH_GSTRING_LEN];
	struct ethtool_regs regs;
	int i;

	ds->ops->get_regs(ds, id, &regs, data);

	for (i = 0; i < count / 2; i++)
		seq_printf(seq, "%2d: %04x\n", i, data[i]);
}

static int dsa_debugfs_regs_read(struct dsa_switch *ds, int id,
				 struct seq_file *seq)
{
	int count;

	if (!ds->ops->get_regs_len || !ds->ops->get_regs)
		return -EOPNOTSUPP;

	count = ds->ops->get_regs_len(ds, id);
	if (count < 0)
		return count;

	dsa_debugfs_regs_read_count(ds, id, seq, count);

	return 0;
}

static const struct dsa_debugfs_ops dsa_debugfs_regs_ops = {
	.read = dsa_debugfs_regs_read,
};

static void dsa_debugfs_stats_read_count(struct dsa_switch *ds, int id,
					 struct seq_file *seq, int count)
{
	u8 strings[count * ETH_GSTRING_LEN];
	u64 stats[count];
	int i;

	ds->ops->get_strings(ds, id, strings);
	ds->ops->get_ethtool_stats(ds, id, stats);

	for (i = 0; i < count; i++)
		seq_printf(seq, "%-20s: %lld\n", strings + i * ETH_GSTRING_LEN,
			   stats[i]);
}

static int dsa_debugfs_stats_read(struct dsa_switch *ds, int id,
				  struct seq_file *seq)
{
	int count;

	if (!ds->ops->get_sset_count || !ds->ops->get_strings ||
	    !ds->ops->get_ethtool_stats)
		return -EOPNOTSUPP;

	count = ds->ops->get_sset_count(ds);
	if (count < 0)
		return count;

	dsa_debugfs_stats_read_count(ds, id, seq, count);

	return 0;
}

static const struct dsa_debugfs_ops dsa_debugfs_stats_ops = {
	.read = dsa_debugfs_stats_read,
};

static int dsa_debugfs_tag_protocol_read(struct dsa_switch *ds, int id,
					 struct seq_file *seq)
{
	enum dsa_tag_protocol proto;

	if (!ds->ops->get_tag_protocol)
		return -EOPNOTSUPP;

	proto = ds->ops->get_tag_protocol(ds);
	seq_printf(seq, "%s\n", dsa_tag_protocol_name(proto));

	return 0;
}

static const struct dsa_debugfs_ops dsa_debugfs_tag_protocol_ops = {
	.read = dsa_debugfs_tag_protocol_read,
};

static int dsa_debugfs_tree_read(struct dsa_switch *ds, int id,
				 struct seq_file *seq)
{
	seq_printf(seq, "%d\n", ds->dst->tree);

	return 0;
}

static const struct dsa_debugfs_ops dsa_debugfs_tree_ops = {
	.read = dsa_debugfs_tree_read,
};

static int dsa_debugfs_vlan_dump_cb(u16 vid, bool pvid, bool untagged,
				    void *data)
{
	struct seq_file *seq = data;

	seq_printf(seq, "vid %d  %s  %s\n", vid,
		   untagged ? "untagged" : "tagged", pvid ? "pvid" : "");

	return 0;
}

static int dsa_debugfs_vlan_read(struct dsa_switch *ds, int id,
				 struct seq_file *seq)
{
	if (!ds->ops->port_vlan_dump)
		return -EOPNOTSUPP;

	return ds->ops->port_vlan_dump(ds, id, dsa_debugfs_vlan_dump_cb, seq);
}

static const struct dsa_debugfs_ops dsa_debugfs_vlan_ops = {
	.read = dsa_debugfs_vlan_read,
};

static int dsa_debugfs_create_port(struct dsa_switch *ds, int port)
{
	struct dentry *dir;
	char name[32];
	int err;

	snprintf(name, sizeof(name), DSA_PORT_FMT, port);

	dir = debugfs_create_dir(name, ds->debugfs_dir);
	if (IS_ERR_OR_NULL(dir))
		return -EFAULT;

	err = dsa_debugfs_create_file(ds, dir, "fdb", port,
				      &dsa_debugfs_fdb_ops);
	if (err)
		return err;

	err = dsa_debugfs_create_file(ds, dir, "mdb", port,
				      &dsa_debugfs_mdb_ops);
	if (err)
		return err;

	err = dsa_debugfs_create_file(ds, dir, "regs", port,
				      &dsa_debugfs_regs_ops);
	if (err)
		return err;

	err = dsa_debugfs_create_file(ds, dir, "stats", port,
				      &dsa_debugfs_stats_ops);
	if (err)
		return err;

	err = dsa_debugfs_create_file(ds, dir, "vlan", port,
				      &dsa_debugfs_vlan_ops);
	if (err)
		return err;

	return 0;
}

static int dsa_debugfs_create_switch(struct dsa_switch *ds)
{
	char name[32];
	int i, err;

	/* skip if there is no debugfs support */
	if (!dsa_debugfs_dir)
		return 0;

	snprintf(name, sizeof(name), DSA_SWITCH_FMT, ds->index);

	ds->debugfs_dir = debugfs_create_dir(name, dsa_debugfs_dir);
	if (IS_ERR_OR_NULL(ds->debugfs_dir))
		return -EFAULT;

	err = dsa_debugfs_create_file(ds, ds->debugfs_dir, "tag_protocol", -1,
				      &dsa_debugfs_tag_protocol_ops);
	if (err)
		return err;

	err = dsa_debugfs_create_file(ds, ds->debugfs_dir, "tree", -1,
				      &dsa_debugfs_tree_ops);
	if (err)
		return err;

	for (i = 0; i < ds->num_ports; i++) {
		if (ds->enabled_port_mask & BIT(i)) {
			err = dsa_debugfs_create_port(ds, i);
			if (err)
				return err;
		}
	}

	return 0;
}

static void dsa_debugfs_destroy_switch(struct dsa_switch *ds)
{
	/* handles NULL */
	debugfs_remove_recursive(ds->debugfs_dir);
}

void dsa_debugfs_create_tree(struct dsa_switch_tree *dst)
{
	struct dsa_switch *ds;
	int i, err;

	for (i = 0; i < DSA_MAX_SWITCHES; i++) {
		ds = dst->ds[i];
		if (!ds)
			continue;

		err = dsa_debugfs_create_switch(ds);
		if (err) {
			pr_warn("DSA: failed to create debugfs interface for switch %d (%d)\n",
				ds->index, err);
			dsa_debugfs_destroy_tree(dst);
			break;
		}
	}
}

void dsa_debugfs_destroy_tree(struct dsa_switch_tree *dst)
{
	struct dsa_switch *ds;
	int i;

	for (i = 0; i < DSA_MAX_SWITCHES; i++) {
		ds = dst->ds[i];
		if (!ds)
			continue;

		dsa_debugfs_destroy_switch(ds);
	}
}

void dsa_debugfs_create_module(void)
{
	dsa_debugfs_dir = debugfs_create_dir("dsa", NULL);
	if (IS_ERR(dsa_debugfs_dir)) {
		pr_warn("DSA: failed to create debugfs interface\n");
		dsa_debugfs_dir = NULL;
	}

	if (dsa_debugfs_dir)
		pr_info("DSA: debugfs interface created\n");
}

void dsa_debugfs_destroy_module(void)
{
	/* handles NULL */
	debugfs_remove_recursive(dsa_debugfs_dir);
}
