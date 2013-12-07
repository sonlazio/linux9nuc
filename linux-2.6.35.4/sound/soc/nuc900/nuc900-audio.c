/*
 * Copyright (c) 2010 Nuvoton technology corporation.
 *
 * Wan ZongShun <mcuos.com@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/ac97.h"
#include "nuc900-audio.h"
#include "../codecs/nau8822.h"

#ifdef CONFIG_SND_SOC_NUC900EVB_AC97
static struct snd_soc_dai_link nuc900evb_ac97_dai = {
        .name		= "AC97",
        .stream_name	= "AC97 HiFi",
        .cpu_dai	= &nuc900_ac97_dai,
        .codec_dai	= &ac97_dai,
};

static struct snd_soc_card nuc900evb_audio_machine = {
        .name		= "NUC900EVB_AC97",
        .dai_link	= &nuc900evb_ac97_dai,
        .num_links	= 1,
        .platform	= &nuc900_soc_platform,
};

static struct snd_soc_device nuc900evb_ac97_devdata = {
        .card		= &nuc900evb_audio_machine,
        .codec_dev	= &soc_codec_dev_ac97,
};
#endif

#ifdef CONFIG_SND_SOC_NUC900EVB_I2S
static int nuc900_audio_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
        struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
        unsigned int sample_rate = params_rate(params);
        int ret;
        unsigned int clk = 0;

        /* set codec DAI configuration */
        ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                  SND_SOC_DAIFMT_NB_NF |
                                  SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;

        /* set cpu DAI configuration */
        ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                  SND_SOC_DAIFMT_NB_NF |
                                  SND_SOC_DAIFMT_CBS_CFS);
        if (ret < 0)
                return ret;

        switch (sample_rate) {
        case 8000:
        case 16000:
        case 32000:
        case 48000:
        case 96000:
                clk = 12288000;
                break;
        case 11025:
        case 22050:
        case 44100:
        case 88200:
                clk = 16934000;
                break;
        }

        /* set the codec system clock for DAC and ADC */
        ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_MCLK, clk, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        /* set MCLK division for sample rate */
        ret = snd_soc_dai_set_sysclk(cpu_dai, NUC900_AUDIO_SAMPLECLK, sample_rate, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        /* set prescaler division for sample rate */
        ret = snd_soc_dai_set_sysclk(cpu_dai, NUC900_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);
        if (ret < 0)
                return ret;

        return 0;
}

static struct snd_soc_ops nuc900_audio_ops = {
        .hw_params = nuc900_audio_hw_params,
};

static struct snd_soc_dai_link nuc900evb_i2s_dai = {
        .name		= "IIS",
        .stream_name	= "IIS HiFi",
        .cpu_dai	= &nuc900_i2s_dai,
        .codec_dai	= &nau8822_dai,
        .ops		= &nuc900_audio_ops,
};

static struct snd_soc_card nuc900evb_audio_machine = {
        .name		= "NUC900EVB_IIS",
        .dai_link	= &nuc900evb_i2s_dai,
        .num_links	= 1,
        .platform	= &nuc900_soc_platform,
};

static struct snd_soc_device nuc900evb_i2s_devdata = {
        .card		= &nuc900evb_audio_machine,
        .codec_dev	= &soc_codec_dev_nau8822,
};
#endif



static struct platform_device *nuc900evb_asoc_dev;

static int __init nuc900evb_audio_init(void)
{
        int ret;

        ret = -ENOMEM;
        nuc900evb_asoc_dev = platform_device_alloc("soc-audio", -1);
        if (!nuc900evb_asoc_dev)
                goto out;

        /* nuc900 board audio device */
#ifdef CONFIG_SND_SOC_NUC900EVB_AC97
        platform_set_drvdata(nuc900evb_asoc_dev, &nuc900evb_ac97_devdata);

        nuc900evb_ac97_devdata.dev = &nuc900evb_asoc_dev->dev;
#endif

#ifdef CONFIG_SND_SOC_NUC900EVB_I2S
        platform_set_drvdata(nuc900evb_asoc_dev, &nuc900evb_i2s_devdata);

        nuc900evb_i2s_devdata.dev = &nuc900evb_asoc_dev->dev;
#endif

        ret = platform_device_add(nuc900evb_asoc_dev);
        if (ret) {
                platform_device_put(nuc900evb_asoc_dev);
                nuc900evb_asoc_dev = NULL;
        }

out:
        return ret;
}

static void __exit nuc900evb_audio_exit(void)
{
        platform_device_unregister(nuc900evb_asoc_dev);
}

module_init(nuc900evb_audio_init);
module_exit(nuc900evb_audio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC900 Series ASoC audio support");
MODULE_AUTHOR("Wan ZongShun");
