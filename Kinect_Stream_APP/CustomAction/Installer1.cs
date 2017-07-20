using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Configuration.Install;
using System.Linq;
using System.Threading.Tasks;

namespace CustomAction
{
  [RunInstaller(true)]
  public partial class Installer1 : System.Configuration.Install.Installer
  {
    public Installer1()
    {
      InitializeComponent();
    }

    public override void Install(System.Collections.IDictionary stateSaver)
    {
      base.Install(stateSaver);

      //// 環境変数「path」を編集
      //string currentPath;
      //currentPath = System.Environment.GetEnvironmentVariable("path", System.EnvironmentVariableTarget.Machine);


      //string installPath = this.Context.Parameters["InstallPath"];

      //string path = installPath.TrimEnd('\\');

      //if (currentPath == null)
      //{
      //  currentPath = path;
      //}
      //else if (currentPath.EndsWith(";"))
      //{
      //  currentPath += path;
      //}
      //else
      //{
      //  currentPath += ";" + path;
      //}

      //// 環境変数を設定する
      //System.Environment.SetEnvironmentVariable("path", currentPath, System.EnvironmentVariableTarget.Machine);
    }

    public override void Commit(IDictionary savedState)
    {
      base.Commit(savedState);
    }

    public override void Rollback(IDictionary savedState)
    {
      base.Rollback(savedState);
    }

    public override void Uninstall(IDictionary savedState)
    {
      base.Uninstall(savedState);

      //// 環境変数「path」を編集
      //string currentPath;
      //currentPath = System.Environment.GetEnvironmentVariable("path", System.EnvironmentVariableTarget.Machine);

      //string installPath = this.Context.Parameters["InstallPath"];
      ////installPath += @"\bin;";
      //installPath.TrimEnd('\\');

      //currentPath = currentPath.Replace(installPath, "");

      //// 環境変数を削除する
      //System.Environment.SetEnvironmentVariable("path", currentPath, System.EnvironmentVariableTarget.Machine);
    }
  }
}
