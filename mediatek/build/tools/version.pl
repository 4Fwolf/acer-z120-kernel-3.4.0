#!perl -w

### location: /mediatek/build/tools/

&usage if ($#ARGV == -1);
#pwd is project root directory
#system 'pwd';

my $pre_offset;
#my $dsp_offset;
my $mbr_offset;
my $ebr1_offset;
my $ebr2_offset;
my $ebr3_offset;
my $ubo_offset;
my $boo_offset;
my $rec_offset; #maybe change depends on fota requirement
my $mob_offset;
my $fat_offset;
my $sys_offset;
my $cus_offset;
my $usr_offset;
my $cac_offset;
my $log_offset;
my $sec_offset;


my $pre;
#my $dsp;
my $mbr;
my $ebr1;
my $ebr2;
my $ebr3;
my $ubo;
my $boo;
my $rec;
my $mob;
my $fat;
my $sys;
my $cus;
my $usr;
my $cac;
my $log;
my $sec;

my $prj = $ARGV[0];
my $outdir = "out/target/product/$prj";

#print "xwguo number:".$#ARGV."\n";
if($#ARGV == 0)
{
	&parse_para();
	&sign();
    #&version();
}
elsif($#ARGV == 1)
{
	&parse_para();
    #&version_preloader();
}

sub version_preloader
{
	if (-e $ARGV[0])
	{
		open FP, "+<$ARGV[0]" or die "can't open preloader image!\n";
		binmode FP;
		seek FP, $pre_offset, 0 or die $!;
		print FP $pre;
		print "preloader image versioned!!!";
		close FP;
	}
}

sub version 
{
	if (-e "$outdir/preloader_$prj.bin")
	{
		open FP, "+<$outdir/preloader_$prj.bin" or die "can't open preloader image!\n";
		binmode FP;
		#seek FP, $pre_offset, 0 or die $!;
		#print FP $pre_version;
		#print "preloader image versioned.\n";
		close FP;
	}

	if (-e "$outdir/DSP_BL")
	{
		open FP, "+<$outdir/DSP_BL" or die "can't open dsp_bl!\n";
		binmode FP;
		#seek FP, $dsp_offset, 0 or die $!;
		#print FP $dsp_version;
		#print "dsp_bl versioned.\n";
		close FP;
	}

	if (-e "$outdir/MBR")
	{
		open FP, "+<$outdir/MBR" or die "can't open mbr!\n";
		binmode FP;
		seek FP, $mbr_offset, 0 or die $!;
		print FP $mbr;
		print "mbr versioned.\n";
		close FP;
	}

	if (-e "$outdir/EBR1")
	{
		open FP, "+<$outdir/EBR1" or die "can't open ebr1!\n";
		binmode FP;
		seek FP, $ebr1_offset, 0 or die $!;
		print FP $ebr1;
		print "ebr1 version.\n";
		close FP;
	}

	if (-e "$outdir/EBR2")
	{
		open FP, "+<$outdir/EBR2" or die "can't open ebr2!\n";
		binmode FP;
		seek FP, $ebr2_offset, 0 or die $!;
		print FP $ebr2;
		print "ebr2 version.\n";
		close FP;
	}

	if (-e "$outdir/EBR3")
	{
		open FP, "+<$outdir/EBR3" or die "can't open ebr3!\n";
		binmode FP;
		seek FP, $ebr3_offset, 0 or die $!;
		print FP $ebr3;
		print "ebr3 version.\n";
		close FP;
	}

	if (-e "$outdir/uboot_$prj.bin")
	{
		open FP, "+<$outdir/uboot_$prj.bin" or die "can't open uboot image!\n";
		binmode FP;
		seek FP, $ubo_offset, 0 or die $!;
		print FP $ubo;
		print "uboot_$prj.bin versioned.\n";
		close FP;
	}

	if (-e "$outdir/logo.bin")
	{
		open FP, "+<$outdir/logo.bin" or die "can't open logo image!\n";
		binmode FP;
		seek FP, $log_offset, 0 or die $!;
		print FP $log;
		print "logo.bin version.\n";
		close FP;
	}

	if (-e "$outdir/boot.img")
	{
		open FP, "+<$outdir/boot.img" or die "can't open boot image!\n";
		binmode FP;
		seek FP, $boo_offset, 0 or die $!;
		print FP $boo;
		print "boot.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/recovery.img")
	{
		open FP, "+<$outdir/recovery.img" or die "can't open recovery image!\n";
		binmode FP;
		seek FP, $rec_offset, 0 or die $!;
		print FP $rec;
		print "recovery.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/fat.img")
	{
		open FP, "+<$outdir/fat.img" or die "can't open fat iamge!\n";
		binmode FP;
		seek FP, $fat_offset, 0 or die $!;
		print FP $fat;
		print "fat.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/mobile_info.img")
	{
		open FP, "+<$outdir/mobile_info.img" or die "can't open mobile info image!\n";
		binmode FP;
		seek FP, $mob_offset, 0 or die $!;
		print FP $mob;
		print "mobile_info.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/system.img")
	{
		open FP, "+<$outdir/system.img" or die "can't open system image!\n";
		binmode FP;
		seek FP, $sys_offset, 0 or die $!;
		print FP $sys;
		print "system.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/custpack.img")
	{
		open FP, "+<$outdir/custpack.img" or die "can't open custpack image!\n";
		binmode FP;
		seek FP, $cus_offset, 0 or die $!;
		print FP $cus;
		print "custpack.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/userdata.img")
	{
		open FP, "+<$outdir/userdata.img" or die "can't open usrdata image!\n";
		binmode FP;
		seek FP, $usr_offset, 0 or die $!;
		print FP $usr;
		print "userdata.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/cache.img")
	{
		open FP, "+<$outdir/cache.img" or die "can't open cache image!\n";
		binmode FP;
		seek FP, $cac_offset, 0 or die $!;
		print FP $cac;
		print "cache.img versioned.\n";
		close FP;
	}

	if (-e "$outdir/secro.img")
	{
		open FP, "+<$outdir/secro.img" or die "can't open secro image!\n";
		binmode FP;
		seek FP, $sec_offset, 0 or die $!;
		print FP $sec;
		print "secro.img versioned.\n";
		close FP;
	}
}

sub parse_para
{
	# parse version number from version.inc
	#system 'pwd';
	#hongmei change from version_megane to development/version
	if ($#ARGV == 0 ){
		open FP, "development/version/include/version.inc" or die "can't open version.inc.\n";
	} elsif ($#ARGV == 1) {
		open FP, "../../development/version/include/version.inc" or die "can't open version.inc.\n";
	}
	@file = <FP>;
	foreach (@file) {
		@array = split;
		if (/pre/i) {
			$pre = substr($array[2], 1, 8);
		}elsif (/mbr/i) {
			$mbr = substr($array[2], 1, 8); 
		}elsif (/ebr1/i) {
			$ebr1 = substr($array[2], 1, 8);
		}elsif (/ebr2/i) {
			$ebr2 = substr($array[2], 1, 8);
		}elsif (/ebr3/i) {
			$ebr3 = substr($array[2], 1, 8);
		}elsif (/ubo/i) {
			$ubo = substr($array[2], 1, 8);
		}elsif (/boo/i) {
			$boo = substr($array[2], 1, 8);
		}elsif (/rec/i) {
			$rec = substr($array[2], 1, 8);
		}elsif (/mob/i) {
			$mob = substr($array[2], 1, 8);
		}elsif (/fat/i) {
			$fat = substr($array[2], 1, 8);
		}elsif (/and/i) {
			$sys = substr($array[2], 1, 8);
		}elsif (/usr/i) {
			$usr  = substr($array[2], 1, 8);
		}elsif (/cus/i) {
			$cus  = substr($array[2], 1, 8);
		}elsif (/cac/i) {
			$cac = substr($array[2], 1, 8);
		}elsif (/log/i) {
			$log = substr($array[2], 1, 8);
#}elsif (/sec/i) {
		}elsif (/sim/i) {
			$sec = substr($array[2], 1, 8);
		}
	}
	close FP;
#print "Test: mbr version number: ".$mbr."\n";

	# parse image offset from version.h
	if ($#ARGV == 0) {
		open FP, "system/extras/version/version.h" or die "can't open version.h.\n";
	} elsif ($#ARGV == 1) {
		open FP, "../../system/extras/version/version.h" or die "can't open version.h\n";
	}
	@file = <FP>;
	foreach (@file) {
		if (/#define/) {
			@array = split;
			if (/pre/i) {
				$pre_offset = hex($array[2]);
				$pre_offset -= 0x800; #minus flashtool inserted header
				$pre_offset -= 0xa0; #minus GFH header
				$pre_offset -= 0x260;
#print "xwguo: preloader offset".$pre_offset."\n";
			}elsif (/mbr/i) {
				$mbr_offset = hex($array[2]); 
			}elsif (/ebr1/i) {
				$ebr1_offset = hex($array[2]);
			}elsif (/ebr2/i) {
				$ebr2_offset = hex($array[2]);
			}elsif (/ebr3/i) {
				$ebr3_offset = hex($array[2]);
			}elsif (/ubo/i) {
				$ubo_offset = hex($array[2]);
			}elsif (/boo/i) {
				$boo_offset = hex($array[2]);
			}elsif (/rec/i) {
				$rec_offset = hex($array[2]);
			}elsif (/mob/i) {
				$mob_offset = hex($array[2]);
			}elsif (/fat/i) {
				$fat_offset = hex($array[2]);
			}elsif (/sys/i) {
				$sys_offset = hex($array[2]);
				$sys_offset += 0x28;
			}elsif (/usr/i) {
				$usr_offset  = hex($array[2]);
				$usr_offset += 0x28;
			}elsif (/cus/i) {
				$cus_offset  = hex($array[2]);
				$cus_offset += 0x28;
			}elsif (/cac/i) {
				$cac_offset = hex($array[2]);
				$cac_offset += 0x28;
			}elsif (/log/i) {
				$log_offset = hex($array[2]);
			}elsif (/sec/i) {
				$sec_offset = hex($array[2]);
				$sec_offset += 0x28;
			}
		}
	}
	close FP;
#print "Test: mbr offset: ".$mbr_offset."\n";
#print "Test: sec offset: ".$sec_offset."\n";
}

sub sign {
	my $us_offset = 0x100;
	my $bs_offset = 0x100;
	my $ss_offset = 0x100;

	my $pattern = "109f10eed3f021e3";

	if (-e "$outdir/uboot_$prj.bin")
	{
		open FP, "+<$outdir/uboot_$prj.bin" or die "can't open uboot image!\n";
		binmode FP;
		seek FP, $us_offset, 0 or die $!;
		print FP $pattern;
		print "uboot_$prj.bin signed.\n";
		close FP;
	}

	if (-e "$outdir/boot.img")
	{
		open FP, "+<$outdir/boot.img" or die "can't open boot image!\n";
		binmode FP;
		seek FP, $bs_offset, 0 or die $!;
		print FP $pattern;
		print "boot.img signed.\n";
		close FP;
	}

	if (-e "$outdir/system.img")
	{
		open FP, "+<$outdir/system.img" or die "can't open system image!\n";
		binmode FP;
		seek FP, $ss_offset, 0 or die $!;
		print FP $pattern;
		print "system.img signed.\n";
		close FP;
	}
}

sub usage
{
	print "usage: perl version.pl <project name>\n";
	exit (0);
}
