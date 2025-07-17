#!/bin/bash
#20180709 V2.0 
###因为要经常切换yum本地源，写了个脚本快速切换，复制全文保存成.sh文件，sh命令执行；
###此脚本可以快速安装阿里，163的yum源，需要root用户执行权限；
###查看CentOS系统版本cat /etc/system-release，确认好版本不要安装错版本的yum源；
###Redhat系统使用阿里，163的源需要清除原有YUM及相关软件包，安装Centos的yum包才能使用yum源；
dir=/etc/yum.repos.d   #Yum source
[ `id -u` -ne 0 ] && echo "Please use the root user to execute $0"&&exit 1
menu(){
echo "Please Correct Input [1-8] Begin install"
echo "-----------------------------------"
echo "Input $0 1 install ftp yum source"
echo "Input $0 2 install iso yum source"
echo "Input $0 3 install aliyun Centos-5.repo"
echo "Input $0 4 install aliyun Centos-6.repo"
echo "Input $0 5 install aliyun Centos-7.repo"
echo "Input $0 6 install 163 CentOS5-Base-163.repo"
echo "Input $0 7 install 163 CentOS6-Base-163.repo"
echo "Input $0 8 install 163 CentOS7-Base-163.repo"
echo "-----------------------------------"
}
backup(){
echo "-----------------------------------"
echo "Begin backup  $dir"
echo "-----------------------------------"
mkdir -p $dir/backup
mv -f $dir/*.repo $dir/backup  
echo "Backup yum files success" 
echo "Backup yum directory "$dir"backup"  
echo "-----------------------------------"
}
clean_yum_cache(){
echo "-----------------------------------"
echo "Begin clean yum all cache"
echo "-----------------------------------"
yum clean all
yum makecache 
echo "-----------------------------------"
echo "$0 script execution end"
echo "-----------------------------------"
}
aliyun_mirrors(){
echo "Testing http://mirrors.aliyun.com/repo/ connectivity"
echo "-----------------------------------"
aliyun=`curl --connect-timeout 10 -I http://mirrors.aliyun.com/repo/|head -n 1|grep 200|wc -l`
  if [ $aliyun -ne 1 ];then
    echo "-----------------------------------"
    echo "mirrors.aliyun.com yum source is unavailable"  
    exit 1
  fi
}
m163_mirrors(){
echo "Testing http://mirrors.163.com/.help/centos.html connectivity"
echo "-----------------------------------"
m163=`curl --connect-timeout 10 -I http://mirrors.163.com/.help/centos.html|head -n 1|grep 200|wc -l`
  if [ $m163 -ne 1 ] ;then
    echo "-----------------------------------"
    echo "mirrors.163.com yum source is unavailable" 
    exit 1
  fi
}
case $1 in
  1 ) 
      read -p "Please enter the ftp Server ipaddress and path:" ip   ###输入FTP服务器IP地址默认使用匿名目录/pub 可以把光盘解压到ftp服务器的/var/ftp/pub目录下作为yum源使用
      ping -c 2 $ip >/dev/null 2>&1
      if [ $? -ne 0 ];then
        echo "Input ipaddress:$ip Host Unreachable"
        echo "Please enter ftp Server ipaddress"
        exit 1
      else
        backup
        cat >  $dir/base.repo << EOF
[base]
name=base
baseurl=ftp://$ip/pub
enabled=1
gpgcheck=0  
EOF
        clean_yum_cache
      fi
    ;;
  2 ) 
      read -p "Please enter the ISO filename path:" iso          ###一般虚拟机物理机iso文件挂载目录为/dev/sr0，这里可以使用iso光盘镜像文件，输入文件的绝对路径名使用，如：/home/centos7.iso
      if [ -z $iso ]; then
        echo "Please enter the ISO filename path:(/dev/sr0)（/home/centos7.iso）"
        exit 1
      elif [ -b $iso ] || [ -f $iso ] ;then
        backup
              if [ ! -d  "/yumiso" ] ;then
                    mkdir -p /yumiso
                else 
                  echo "/yumiso mount directory already exists"
              fi
        mount -o loop $iso /yumiso
        cat > $dir/base.repo <<EOF
[base]
name=base
baseurl=file:///yumiso
enabled=1
gpgcheck=0
EOF
      else         
        echo "Please enter the ISO filename path:(/dev/sr0)（/home/centos7.iso）"
        echo "$iso filename or path does not exist"
        exit 1
      fi
      clean_yum_cache
    ;;
  3 ) 
      aliyun_mirrors
      backup
      cd $dir
      wget http://mirrors.aliyun.com/repo/Centos-5.repo
      clean_yum_cache
    ;;
  4 ) 
      aliyun_mirrors
      backup
      cd $dir
      wget http://mirrors.aliyun.com/repo/Centos-6.repo
      clean_yum_cache
    ;;
  5 ) 
      aliyun_mirrors
      backup
      cd $dir
      wget http://mirrors.aliyun.com/repo/Centos-7.repo
      clean_yum_cache
    ;;
  6 ) 
      m163_mirrors
      backup
      cd $dir
      wget http://mirrors.163.com/.help/CentOS5-Base-163.repo
      clean_yum_cache
    ;;
  7 ) 
      m163_mirrors
      backup
      cd $dir
      wget http://mirrors.163.com/.help/CentOS6-Base-163.repo
      clean_yum_cache
    ;;
  8 ) 
      m163_mirrors
      backup
      cd $dir
      wget http://mirrors.163.com/.help/CentOS7-Base-163.repo
      clean_yum_cache
    ;;
  * )
      menu
      exit
    ;;
esac